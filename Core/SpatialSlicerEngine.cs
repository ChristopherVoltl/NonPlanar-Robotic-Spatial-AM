using System;
using System.Collections.Generic;
using System.Linq;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class SpatialSlicerEngine
{
    public SliceResult Generate(IReadOnlyList<TopologySection> sections, SliceGenerationOptions options)
    {
        if (sections is null)
        {
            throw new ArgumentNullException(nameof(sections));
        }

        if (options is null)
        {
            throw new ArgumentNullException(nameof(options));
        }

        if (sections.Count == 0)
        {
            return new SliceResult(Array.Empty<SlicePath>(), BuildAnalytics(Array.Empty<SlicePath>()));
        }

        var nodes = BuildGraph(sections);
        PopulateTopologyField(nodes, options);
        var orderedNodes = ExtractAdaptiveOrder(nodes, options);

        var slices = new List<SlicePath>(orderedNodes.Count);
        Vector3D? previousNormal = null;
        int sliceIndex = 0;

        foreach (var node in orderedNodes)
        {
            var frame = BuildFrame(node, previousNormal, options);
            var segments = node.Section.GuidePath.ToSegments();
            slices.Add(new SlicePath(sliceIndex++, node.Section, frame, segments));
            previousNormal = frame.Normal;
        }

        return new SliceResult(slices, BuildAnalytics(slices));
    }

    private static List<SectionNode> BuildGraph(IReadOnlyList<TopologySection> sections)
    {
        var nodes = sections
            .OrderBy(section => section.Index)
            .Select(section => new SectionNode(section, ComputeCentroid(section.GuidePath)))
            .ToList();

        var modelScale = ComputeModelScale(nodes);
        var endpointTolerance = Math.Max(modelScale * 0.025, 1e-4);

        for (int i = 0; i < nodes.Count; i++)
        {
            for (int j = i + 1; j < nodes.Count; j++)
            {
                if (AreAdjacent(nodes[i], nodes[j], endpointTolerance))
                {
                    nodes[i].Neighbors.Add(nodes[j]);
                    nodes[j].Neighbors.Add(nodes[i]);
                }
            }
        }

        foreach (var node in nodes.Where(node => node.Neighbors.Count == 0))
        {
            foreach (var nearest in nodes
                .Where(other => !ReferenceEquals(other, node))
                .OrderBy(other => Distance(node.Centroid, other.Centroid))
                .Take(Math.Min(2, nodes.Count - 1)))
            {
                if (!node.Neighbors.Contains(nearest))
                {
                    node.Neighbors.Add(nearest);
                }

                if (!nearest.Neighbors.Contains(node))
                {
                    nearest.Neighbors.Add(node);
                }
            }
        }

        return nodes;
    }

    private static void PopulateTopologyField(List<SectionNode> nodes, SliceGenerationOptions options)
    {
        var boundaryNodes = nodes.Where(node => node.Neighbors.Count <= 1).ToList();
        if (boundaryNodes.Count == 0)
        {
            boundaryNodes = nodes.OrderBy(node => node.Centroid.Z).Take(2).ToList();
        }

        var saddleNodes = nodes
            .Where(node => node.Neighbors.Count >= 3 || AverageNormalDivergence(node) >= options.SaddleNormalThresholdDegrees)
            .ToList();
        if (saddleNodes.Count == 0)
        {
            saddleNodes = nodes.OrderByDescending(AverageNormalDivergence).Take(Math.Min(2, nodes.Count)).ToList();
        }

        var boundaryDistances = ComputeMultiSourceDistances(nodes, boundaryNodes);
        var saddleDistances = ComputeMultiSourceDistances(nodes, saddleNodes);
        var normalizedBoundary = NormalizeDistances(nodes, boundaryDistances);
        var normalizedSaddle = NormalizeDistances(nodes, saddleDistances);

        foreach (var node in nodes)
        {
            node.BoundaryDistance = normalizedBoundary[node];
            node.SaddleDistance = normalizedSaddle[node];
        }

        foreach (var node in nodes)
        {
            double saddleKernel = ComputeSaddleKernel(node, saddleNodes, options.SaddleBiasExponent);
            double saddleBias = options.SaddleBiasStrength * saddleKernel * (1.0 - node.SaddleDistance);
            node.FieldValue = options.ClampUnit(node.BoundaryDistance + saddleBias);
        }

        SmoothField(nodes, options.SmoothingFactor);
        EstimateGradients(nodes);
    }

    private static List<SectionNode> ExtractAdaptiveOrder(List<SectionNode> nodes, SliceGenerationOptions options)
    {
        var orderedByField = nodes.OrderBy(node => node.FieldValue).ThenBy(node => node.Section.Index).ToList();
        var output = new List<SectionNode>(orderedByField.Count);
        var assigned = new HashSet<SectionNode>();

        double modelScale = ComputeModelScale(nodes);
        double uniformStep = options.ClampScalarStep(options.TargetLayerHeight / Math.Max(modelScale, 1.0));
        double tau = 0.0;

        while (tau < 1.0 + NumericalTolerance.Epsilon)
        {
            double localGradient = EstimateLocalGradient(orderedByField, tau);
            double adaptiveStep = uniformStep / Math.Max(0.35, 1.0 + (localGradient * options.TargetLayerHeight));
            double step = options.ClampScalarStep(
                (options.AdaptiveBlendFactor * uniformStep) + ((1.0 - options.AdaptiveBlendFactor) * adaptiveStep));

            double upper = Math.Min(1.0 + NumericalTolerance.Epsilon, tau + step);
            var bandNodes = orderedByField
                .Where(node => !assigned.Contains(node) && node.FieldValue >= tau - NumericalTolerance.Epsilon && node.FieldValue < upper)
                .OrderBy(node => node.FieldValue)
                .ThenBy(node => node.Section.Index)
                .ToList();

            foreach (var node in bandNodes)
            {
                assigned.Add(node);
                output.Add(node);
            }

            tau = upper;
            if (tau >= 1.0)
            {
                break;
            }
        }

        foreach (var node in orderedByField.Where(node => !assigned.Contains(node)))
        {
            output.Add(node);
        }

        return output;
    }

    private static SliceFrame BuildFrame(SectionNode node, Vector3D? previousNormal, SliceGenerationOptions options)
    {
        Vector3D globalUp = options.GlobalUpAxis.Unitized();
        Vector3D topologyNormal = node.Section.TopologyNormal.Unitized();
        double rawAngle = topologyNormal.AngleTo(globalUp);

        double fieldInfluence = options.ClampUnit(node.FieldValue + (options.SaddleBiasStrength * (1.0 - node.SaddleDistance)));
        double scaledAngle = Math.Max(0.0, Math.Min(180.0, rawAngle * options.NormalizedAngleInfluence * fieldInfluence));

        Vector3D targetNormal = Vector3D.Lerp(globalUp, topologyNormal, scaledAngle / 180.0);
        if (previousNormal.HasValue)
        {
            targetNormal = Vector3D.Lerp(previousNormal.Value, targetNormal, 1.0 - options.SmoothingFactor);
        }

        AxisAngleSet axisAngles = new AxisAngleSet(
            targetNormal.AngleTo(Vector3D.XAxis),
            targetNormal.AngleTo(Vector3D.YAxis),
            targetNormal.AngleTo(Vector3D.ZAxis));

        return new SliceFrame(node.Centroid, targetNormal, targetNormal.AngleTo(globalUp), axisAngles);
    }

    private static Dictionary<SectionNode, double> ComputeMultiSourceDistances(List<SectionNode> nodes, List<SectionNode> sources)
    {
        var distances = nodes.ToDictionary(node => node, _ => double.PositiveInfinity);
        var frontier = new List<SectionNode>(sources);

        foreach (var source in sources)
        {
            distances[source] = 0.0;
        }

        while (frontier.Count > 0)
        {
            SectionNode current = frontier.OrderBy(node => distances[node]).First();
            frontier.Remove(current);

            foreach (SectionNode neighbor in current.Neighbors)
            {
                double edgeWeight = Distance(current.Centroid, neighbor.Centroid) + (0.25 * (current.MeanSegmentLength + neighbor.MeanSegmentLength));
                double candidate = distances[current] + edgeWeight;
                if (candidate + NumericalTolerance.Epsilon < distances[neighbor])
                {
                    distances[neighbor] = candidate;
                    if (!frontier.Contains(neighbor))
                    {
                        frontier.Add(neighbor);
                    }
                }
            }
        }

        return distances;
    }

    private static Dictionary<SectionNode, double> NormalizeDistances(List<SectionNode> nodes, Dictionary<SectionNode, double> distances)
    {
        double maxFinite = distances.Values.Where(value => !double.IsInfinity(value)).DefaultIfEmpty(1.0).Max();
        if (maxFinite <= NumericalTolerance.Epsilon)
        {
            maxFinite = 1.0;
        }

        return nodes.ToDictionary(
            node => node,
            node => double.IsInfinity(distances[node]) ? 1.0 : distances[node] / maxFinite);
    }

    private static double ComputeSaddleKernel(SectionNode node, List<SectionNode> saddles, double exponent)
    {
        if (saddles.Count == 0)
        {
            return 0.0;
        }

        double sum = 0.0;
        foreach (var saddle in saddles)
        {
            double distance = Distance(node.Centroid, saddle.Centroid);
            sum += 1.0 / Math.Pow(distance + 1.0, Math.Max(0.1, exponent));
        }

        double selfMax = saddles.Max(saddle => 1.0 / Math.Pow(Distance(saddle.Centroid, saddle.Centroid) + 1.0, Math.Max(0.1, exponent)));
        double theoreticalMax = saddles.Count * Math.Max(1.0, selfMax);
        return Math.Max(0.0, Math.Min(1.0, sum / theoreticalMax));
    }

    private static void SmoothField(List<SectionNode> nodes, double smoothingFactor)
    {
        double blend = Math.Max(0.0, Math.Min(1.0, smoothingFactor));
        for (int iteration = 0; iteration < 2; iteration++)
        {
            var smoothed = nodes.ToDictionary(
                node => node,
                node => node.Neighbors.Count == 0
                    ? node.FieldValue
                    : ((1.0 - blend) * node.FieldValue) + (blend * node.Neighbors.Average(neighbor => neighbor.FieldValue)));

            foreach (var node in nodes)
            {
                node.FieldValue = smoothed[node];
            }
        }
    }

    private static void EstimateGradients(List<SectionNode> nodes)
    {
        foreach (var node in nodes)
        {
            if (node.Neighbors.Count == 0)
            {
                node.FieldGradient = 0.0;
                continue;
            }

            node.FieldGradient = node.Neighbors
                .Select(neighbor => Math.Abs(node.FieldValue - neighbor.FieldValue) / Math.Max(Distance(node.Centroid, neighbor.Centroid), NumericalTolerance.Epsilon))
                .Average();
        }
    }

    private static double EstimateLocalGradient(List<SectionNode> orderedNodes, double tau)
    {
        var nearby = orderedNodes
            .Where(node => Math.Abs(node.FieldValue - tau) <= 0.08)
            .Select(node => node.FieldGradient)
            .ToList();

        if (nearby.Count == 0)
        {
            nearby = orderedNodes.Select(node => node.FieldGradient).ToList();
        }

        return nearby.DefaultIfEmpty(0.0).Average();
    }

    private static double AverageNormalDivergence(SectionNode node)
    {
        if (node.Neighbors.Count == 0)
        {
            return 0.0;
        }

        return node.Neighbors.Average(neighbor => node.Section.TopologyNormal.AngleTo(neighbor.Section.TopologyNormal));
    }

    private static bool AreAdjacent(SectionNode a, SectionNode b, double tolerance)
    {
        Point3D aStart = a.Section.GuidePath.Points.First();
        Point3D aEnd = a.Section.GuidePath.Points.Last();
        Point3D bStart = b.Section.GuidePath.Points.First();
        Point3D bEnd = b.Section.GuidePath.Points.Last();

        double minEndpointDistance = new[]
        {
            Distance(aStart, bStart),
            Distance(aStart, bEnd),
            Distance(aEnd, bStart),
            Distance(aEnd, bEnd)
        }.Min();

        if (minEndpointDistance <= tolerance)
        {
            return true;
        }

        double centroidDistance = Distance(a.Centroid, b.Centroid);
        double normalAngle = a.Section.TopologyNormal.AngleTo(b.Section.TopologyNormal);
        return centroidDistance <= tolerance * 2.5 && normalAngle <= 75.0;
    }

    private static Point3D ComputeCentroid(Polyline3D polyline)
    {
        if (polyline.Points.Count == 0)
        {
            return new Point3D(0.0, 0.0, 0.0);
        }

        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        foreach (Point3D point in polyline.Points)
        {
            sumX += point.X;
            sumY += point.Y;
            sumZ += point.Z;
        }

        double count = polyline.Points.Count;
        return new Point3D(sumX / count, sumY / count, sumZ / count);
    }

    private static double ComputeModelScale(List<SectionNode> nodes)
    {
        double minX = nodes.Min(node => node.Centroid.X);
        double minY = nodes.Min(node => node.Centroid.Y);
        double minZ = nodes.Min(node => node.Centroid.Z);
        double maxX = nodes.Max(node => node.Centroid.X);
        double maxY = nodes.Max(node => node.Centroid.Y);
        double maxZ = nodes.Max(node => node.Centroid.Z);
        return Math.Sqrt(Math.Pow(maxX - minX, 2) + Math.Pow(maxY - minY, 2) + Math.Pow(maxZ - minZ, 2));
    }

    private static double Distance(Point3D a, Point3D b)
    {
        return (a - b).Length;
    }

    private static SliceAnalytics BuildAnalytics(IReadOnlyList<SlicePath> slices)
    {
        var allSegments = slices.SelectMany(slice => slice.Segments).ToList();
        var segmentLengths = allSegments.Select(segment => segment.Length).DefaultIfEmpty(0.0).ToList();
        var upAngles = slices.Select(slice => slice.Frame.AngleFromGlobalUpDegrees).DefaultIfEmpty(0.0).ToList();
        var xAngles = slices.Select(slice => slice.Frame.AxisAngles.XDegrees).DefaultIfEmpty(0.0).ToList();
        var yAngles = slices.Select(slice => slice.Frame.AxisAngles.YDegrees).DefaultIfEmpty(0.0).ToList();
        var zAngles = slices.Select(slice => slice.Frame.AxisAngles.ZDegrees).DefaultIfEmpty(0.0).ToList();

        return new SliceAnalytics(
            new SegmentLengthStatistics(segmentLengths.Min(), segmentLengths.Max(), segmentLengths.Average()),
            new AngleStatistics(upAngles.Min(), upAngles.Max(), upAngles.Average()),
            new AngleStatistics(xAngles.Min(), xAngles.Max(), xAngles.Average()),
            new AngleStatistics(yAngles.Min(), yAngles.Max(), yAngles.Average()),
            new AngleStatistics(zAngles.Min(), zAngles.Max(), zAngles.Average()));
    }

    private sealed class SectionNode
    {
        public SectionNode(TopologySection section, Point3D centroid)
        {
            Section = section;
            Centroid = centroid;
            Neighbors = new List<SectionNode>();
            MeanSegmentLength = section.GuidePath.ToSegments().Select(segment => segment.Length).DefaultIfEmpty(0.0).Average();
        }

        public TopologySection Section { get; }
        public Point3D Centroid { get; }
        public List<SectionNode> Neighbors { get; }
        public double MeanSegmentLength { get; }
        public double BoundaryDistance { get; set; }
        public double SaddleDistance { get; set; }
        public double FieldValue { get; set; }
        public double FieldGradient { get; set; }
    }
}
