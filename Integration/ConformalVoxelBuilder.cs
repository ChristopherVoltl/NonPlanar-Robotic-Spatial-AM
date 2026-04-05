using System;
using System.Collections.Generic;
using System.Linq;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Defines the sizing and normalization controls for conformal voxel generation.
    /// </summary>
    /// <remarks>
    /// This options type exists so the Grasshopper component can stay thin while the builder remains reusable.
    /// Preconditions: size values should be positive and <see cref="MinVoxelSize"/> should not exceed
    /// <see cref="MaxVoxelSize"/>. Side-effects: none.
    /// </remarks>
    internal sealed class ConformalVoxelOptions
    {
        public double FallbackSegmentLength { get; set; } = 10.0;
        public double MinVoxelSize { get; set; } = 5.0;
        public double MaxVoxelSize { get; set; } = 15.0;
        public Curve? StartGuideCurve { get; set; }
        public Curve? DirectionGuideCurve { get; set; }
    }

    /// <summary>
    /// Carries the conformal voxel scaffold created from an ordered layer stack.
    /// </summary>
    internal sealed class ConformalVoxelResult
    {
        public ConformalVoxelResult(
            IReadOnlyList<Curve> ringCurves,
            IReadOnlyList<Line> voxelEdges,
            IReadOnlyList<Point3d> voxelCenters,
            IReadOnlyList<Curve> faceLoops,
            IReadOnlyList<Point3d> nodes,
            int sharedPerimeterDivisionCount,
            int sharedRadialBandCount,
            string analysis)
        {
            RingCurves = ringCurves;
            VoxelEdges = voxelEdges;
            VoxelCenters = voxelCenters;
            FaceLoops = faceLoops;
            Nodes = nodes;
            SharedPerimeterDivisionCount = sharedPerimeterDivisionCount;
            SharedRadialBandCount = sharedRadialBandCount;
            Analysis = analysis;
        }

        public IReadOnlyList<Curve> RingCurves { get; }
        public IReadOnlyList<Line> VoxelEdges { get; }
        public IReadOnlyList<Point3d> VoxelCenters { get; }
        public IReadOnlyList<Curve> FaceLoops { get; }
        public IReadOnlyList<Point3d> Nodes { get; }
        public int SharedPerimeterDivisionCount { get; }
        public int SharedRadialBandCount { get; }
        public string Analysis { get; }
    }

    /// <summary>
    /// Builds a conformal voxel scaffold from ordered nonplanar layer curves.
    /// </summary>
    /// <remarks>
    /// This builder differs from <see cref="SpatialLatticeBuilder"/> by focusing on normalized voxel bands and cell
    /// centers rather than truss bracing patterns. Input order defines stack order. Side-effects: none beyond
    /// allocating temporary Rhino geometry for the current solve.
    /// </remarks>
    internal static class ConformalVoxelBuilder
    {
        /// <summary>
        /// Converts ordered layer curves into shared ring divisions, voxel cells, and diagnostic geometry.
        /// </summary>
        /// <param name="layerCurves">Closed layer curves ordered through the stack. Cannot be <see langword="null"/> or empty.</param>
        /// <param name="layerPlanes">Optional layer planes matching the curve order. May be empty.</param>
        /// <param name="options">Sizing and guide settings. Cannot be <see langword="null"/>.</param>
        /// <param name="tolerance">Geometric tolerance used for offsets and duplicate filtering. Must be greater than zero.</param>
        /// <returns>
        /// A <see cref="ConformalVoxelResult"/>. When no valid loops are available the geometry collections are empty and
        /// the analysis text explains the failure.
        /// </returns>
        /// <exception cref="ArgumentNullException">Thrown when <paramref name="options"/> is <see langword="null"/>.</exception>
        public static ConformalVoxelResult Build(
            IReadOnlyList<Curve> layerCurves,
            IReadOnlyList<Plane> layerPlanes,
            ConformalVoxelOptions options,
            double tolerance)
        {
            if (options == null)
            {
                throw new ArgumentNullException(nameof(options));
            }

            var ringCurves = new List<Curve>();
            var voxelEdges = new List<Line>();
            var voxelCenters = new List<Point3d>();
            var faceLoops = new List<Curve>();
            var notes = new List<string>();

            if (layerCurves == null || layerCurves.Count == 0)
            {
                return new ConformalVoxelResult(
                    ringCurves,
                    voxelEdges,
                    voxelCenters,
                    faceLoops,
                    Array.Empty<Point3d>(),
                    0,
                    0,
                    "No layer curves were supplied. Input order defines stack order for this component.");
            }

            double safeTolerance = Math.Max(tolerance, 1e-6);
            double minVoxelSize = Math.Max(options.MinVoxelSize, safeTolerance * 4.0);
            double maxVoxelSize = Math.Max(options.MaxVoxelSize, minVoxelSize);

            var layers = new List<LayerData>();
            for (int index = 0; index < layerCurves.Count; index++)
            {
                Plane? plane = index < layerPlanes.Count ? layerPlanes[index] : (Plane?)null;
                LayerData? layer = TryBuildLayer(index, layerCurves[index], plane, options, safeTolerance, notes);
                if (layer != null)
                {
                    layers.Add(layer);
                }
            }

            if (layers.Count == 0)
            {
                return new ConformalVoxelResult(
                    ringCurves,
                    voxelEdges,
                    voxelCenters,
                    faceLoops,
                    Array.Empty<Point3d>(),
                    0,
                    0,
                    "No valid closed layers were available for voxel generation.");
            }

            int sharedPerimeterDivisions = DetermineSharedPerimeterDivisionCount(layers, minVoxelSize, maxVoxelSize, notes);
            NormalizeLayerStack(layers, sharedPerimeterDivisions, options.StartGuideCurve, options.DirectionGuideCurve, safeTolerance, notes);

            int sharedRadialBands = DetermineSharedRadialBandCount(layers, minVoxelSize, maxVoxelSize, safeTolerance, notes);
            foreach (LayerData layer in layers)
            {
                BuildLayerRings(layer, sharedRadialBands, safeTolerance, notes);
                ringCurves.AddRange(layer.RingCurves);
            }

            BuildVoxelScaffold(layers, voxelEdges, voxelCenters, faceLoops, safeTolerance);
            IReadOnlyList<Point3d> nodes = CollectUniqueNodes(layers, voxelEdges, safeTolerance);

            string analysis = BuildAnalysis(
                layers,
                sharedPerimeterDivisions,
                sharedRadialBands,
                nodes.Count,
                voxelEdges.Count,
                voxelCenters.Count,
                faceLoops.Count,
                notes);

            return new ConformalVoxelResult(
                ringCurves,
                voxelEdges,
                voxelCenters,
                faceLoops,
                nodes,
                sharedPerimeterDivisions,
                sharedRadialBands,
                analysis);
        }

        private static LayerData? TryBuildLayer(
            int stackIndex,
            Curve curve,
            Plane? suppliedPlane,
            ConformalVoxelOptions options,
            double tolerance,
            ICollection<string> notes)
        {
            if (curve == null || !curve.IsValid)
            {
                notes.Add("Skipped an invalid layer curve.");
                return null;
            }

            List<Point3d> perimeter = ExtractClosedLayerPoints(curve, options.FallbackSegmentLength, tolerance);
            if (perimeter.Count < 3)
            {
                notes.Add("Skipped a layer because it was not a usable closed loop.");
                return null;
            }

            Plane plane = ResolveLayerPlane(perimeter, suppliedPlane);
            return new LayerData(stackIndex, plane, perimeter);
        }

        private static int DetermineSharedPerimeterDivisionCount(
            IReadOnlyList<LayerData> layers,
            double minVoxelSize,
            double maxVoxelSize,
            ICollection<string> notes)
        {
            int lowerBound = 3;
            int upperBound = int.MaxValue;
            double averageLength = 0.0;

            foreach (LayerData layer in layers)
            {
                double length = ComputeClosedLength(layer.PerimeterPoints);
                averageLength += length;
                lowerBound = Math.Max(lowerBound, (int)Math.Ceiling(length / maxVoxelSize));
                upperBound = Math.Min(upperBound, Math.Max(3, (int)Math.Floor(length / minVoxelSize)));
            }

            averageLength /= Math.Max(1, layers.Count);
            int target = Math.Max(3, (int)Math.Round(averageLength / ((minVoxelSize + maxVoxelSize) * 0.5)));

            if (upperBound >= lowerBound)
            {
                return Math.Min(upperBound, Math.Max(lowerBound, target));
            }

            notes.Add("Perimeter voxel-size limits conflicted across layers, so the shared division count was biased toward the upper voxel size bound.");
            return lowerBound;
        }

        private static int DetermineSharedRadialBandCount(
            IReadOnlyList<LayerData> layers,
            double minVoxelSize,
            double maxVoxelSize,
            double tolerance,
            ICollection<string> notes)
        {
            double smallestRadius = double.MaxValue;
            foreach (LayerData layer in layers)
            {
                double radius = EstimateLayerUsableRadius(layer, tolerance);
                if (radius > tolerance)
                {
                    smallestRadius = Math.Min(smallestRadius, radius);
                }
            }

            if (double.IsNaN(smallestRadius) || double.IsInfinity(smallestRadius) || smallestRadius <= tolerance)
            {
                notes.Add("The tightest layer radius only supported a single voxel band.");
                return 1;
            }

            int lowerBound = Math.Max(1, (int)Math.Ceiling(smallestRadius / maxVoxelSize));
            int upperBound = Math.Max(1, (int)Math.Floor(smallestRadius / minVoxelSize));
            int target = Math.Max(1, (int)Math.Round(smallestRadius / ((minVoxelSize + maxVoxelSize) * 0.5)));

            if (upperBound >= lowerBound)
            {
                return Math.Min(upperBound, Math.Max(lowerBound, target));
            }

            notes.Add("Radial voxel-size limits conflicted with the smallest layer radius, so the band count was biased toward the upper voxel size bound.");
            return lowerBound;
        }

        private static void NormalizeLayerStack(
            IReadOnlyList<LayerData> layers,
            int sharedPerimeterDivisions,
            Curve? startGuideCurve,
            Curve? directionGuideCurve,
            double tolerance,
            ICollection<string> notes)
        {
            foreach (LayerData layer in layers)
            {
                double seamParameter = ResolveLayerSeamParameter(layer, startGuideCurve, tolerance);
                layer.NormalizePerimeter(sharedPerimeterDivisions, seamParameter);
            }

            if (layers.Count == 0)
            {
                return;
            }

            Vector3d desiredFirstDirection = ResolveGuideDirection(layers[0].Plane, directionGuideCurve, tolerance);
            if (desiredFirstDirection.IsTiny())
            {
                desiredFirstDirection = ResolveGuideDirection(layers[0].Plane, startGuideCurve, tolerance);
            }

            Vector3d actualFirstDirection = layers[0].GetStartTraversalDirection();
            if (!desiredFirstDirection.IsTiny() &&
                !actualFirstDirection.IsTiny() &&
                (desiredFirstDirection * actualFirstDirection) < 0.0)
            {
                layers[0].ReversePreservingStart();
                notes.Add("Flipped normalized direction on layer 1 to match the guide direction.");
            }

            for (int index = 1; index < layers.Count; index++)
            {
                double asIsScore = ComputeSecondPointVerticalityScore(layers[index - 1], layers[index], false);
                double reversedScore = ComputeSecondPointVerticalityScore(layers[index - 1], layers[index], true);
                if (reversedScore + Math.Max(tolerance, 1e-6) < asIsScore)
                {
                    layers[index].ReversePreservingStart();
                    notes.Add("Flipped normalized direction on layer " + (layers[index].StackIndex + 1) + " because the second-point test produced a more vertical correspondence.");
                }
            }
        }

        private static void BuildLayerRings(
            LayerData layer,
            int sharedRadialBands,
            double tolerance,
            ICollection<string> notes)
        {
            layer.RebuildProjectedData();
            layer.Rings.Clear();
            layer.RingCurves.Clear();
            layer.Rings.Add(new List<Point3d>(layer.PerimeterPoints));
            layer.RingCurves.Add(CreateClosedPolylineCurve(layer.PerimeterPoints));

            if (sharedRadialBands <= 0)
            {
                return;
            }

            double usableRadius = EstimateLayerUsableRadius(layer, tolerance);
            double stepDistance = Math.Max(tolerance * 4.0, usableRadius / (sharedRadialBands + 1.0));
            Curve projectedPerimeterCurve = CreateClosedPolylineCurve(layer.ProjectedPerimeter);

            for (int bandIndex = 1; bandIndex <= sharedRadialBands; bandIndex++)
            {
                double offsetDistance = stepDistance * bandIndex;
                Curve? inwardOffset = TryCreateInwardOffset(projectedPerimeterCurve, layer.Plane, offsetDistance, tolerance);
                List<Point3d> ring = inwardOffset != null
                    ? BuildOffsetDrivenInteriorRing(layer, inwardOffset, offsetDistance, tolerance)
                    : BuildScaledInteriorRing(layer, (sharedRadialBands + 1.0 - bandIndex) / (sharedRadialBands + 1.0), tolerance);

                ring = RemoveSequentialDuplicates(ring, tolerance);
                if (ring.Count < 3 || ComputeAverageClosedSegmentLength(ring) <= tolerance)
                {
                    notes.Add("Stopped inward voxel bands on layer " + (layer.StackIndex + 1) + " because the next inner ring collapsed below tolerance.");
                    break;
                }

                if (ring.Count != layer.PerimeterPoints.Count)
                {
                    ring = new TopologicalRingSampler(ring).Sample(layer.PerimeterPoints.Count);
                }

                layer.Rings.Add(ring);
                layer.RingCurves.Add(CreateClosedPolylineCurve(ring));
            }
        }

        private static void BuildVoxelScaffold(
            IReadOnlyList<LayerData> layers,
            ICollection<Line> voxelEdges,
            ICollection<Point3d> voxelCenters,
            ICollection<Curve> faceLoops,
            double tolerance)
        {
            var edgeKeys = new HashSet<string>(StringComparer.Ordinal);
            var faceKeys = new HashSet<string>(StringComparer.Ordinal);

            foreach (LayerData layer in layers)
            {
                for (int ringIndex = 0; ringIndex < layer.Rings.Count; ringIndex++)
                {
                    IReadOnlyList<Point3d> ring = layer.Rings[ringIndex];
                    for (int pointIndex = 0; pointIndex < ring.Count; pointIndex++)
                    {
                        TryAddEdge(ring[pointIndex], ring[(pointIndex + 1) % ring.Count], voxelEdges, edgeKeys, tolerance);
                    }

                    if (ringIndex < layer.Rings.Count - 1)
                    {
                        IReadOnlyList<Point3d> innerRing = layer.Rings[ringIndex + 1];
                        for (int pointIndex = 0; pointIndex < ring.Count; pointIndex++)
                        {
                            TryAddEdge(ring[pointIndex], innerRing[pointIndex], voxelEdges, edgeKeys, tolerance);
                        }
                    }
                }
            }

            for (int layerIndex = 0; layerIndex < layers.Count - 1; layerIndex++)
            {
                LayerData lower = layers[layerIndex];
                LayerData upper = layers[layerIndex + 1];
                int ringCount = Math.Min(lower.Rings.Count, upper.Rings.Count);

                for (int ringIndex = 0; ringIndex < ringCount; ringIndex++)
                {
                    IReadOnlyList<Point3d> lowerRing = lower.Rings[ringIndex];
                    IReadOnlyList<Point3d> upperRing = upper.Rings[ringIndex];
                    int pointCount = Math.Min(lowerRing.Count, upperRing.Count);
                    for (int pointIndex = 0; pointIndex < pointCount; pointIndex++)
                    {
                        TryAddEdge(lowerRing[pointIndex], upperRing[pointIndex], voxelEdges, edgeKeys, tolerance);
                    }
                }

                for (int bandIndex = 0; bandIndex < ringCount - 1; bandIndex++)
                {
                    IReadOnlyList<Point3d> lowerOuter = lower.Rings[bandIndex];
                    IReadOnlyList<Point3d> lowerInner = lower.Rings[bandIndex + 1];
                    IReadOnlyList<Point3d> upperOuter = upper.Rings[bandIndex];
                    IReadOnlyList<Point3d> upperInner = upper.Rings[bandIndex + 1];
                    int pointCount = Math.Min(Math.Min(lowerOuter.Count, lowerInner.Count), Math.Min(upperOuter.Count, upperInner.Count));

                    for (int pointIndex = 0; pointIndex < pointCount; pointIndex++)
                    {
                        int nextIndex = (pointIndex + 1) % pointCount;
                        Point3d p000 = lowerOuter[pointIndex];
                        Point3d p010 = lowerOuter[nextIndex];
                        Point3d p110 = lowerInner[nextIndex];
                        Point3d p100 = lowerInner[pointIndex];
                        Point3d p001 = upperOuter[pointIndex];
                        Point3d p011 = upperOuter[nextIndex];
                        Point3d p111 = upperInner[nextIndex];
                        Point3d p101 = upperInner[pointIndex];

                        voxelCenters.Add(AveragePoint(new[] { p000, p010, p110, p100, p001, p011, p111, p101 }));

                        TryAddEdge(p000, p010, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p010, p110, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p110, p100, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p100, p000, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p001, p011, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p011, p111, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p111, p101, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p101, p001, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p000, p001, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p010, p011, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p110, p111, voxelEdges, edgeKeys, tolerance);
                        TryAddEdge(p100, p101, voxelEdges, edgeKeys, tolerance);

                        TryAddFace(new[] { p000, p010, p110, p100 }, faceLoops, faceKeys, tolerance);
                        TryAddFace(new[] { p001, p011, p111, p101 }, faceLoops, faceKeys, tolerance);
                        TryAddFace(new[] { p000, p010, p011, p001 }, faceLoops, faceKeys, tolerance);
                        TryAddFace(new[] { p100, p110, p111, p101 }, faceLoops, faceKeys, tolerance);
                        TryAddFace(new[] { p000, p100, p101, p001 }, faceLoops, faceKeys, tolerance);
                        TryAddFace(new[] { p010, p110, p111, p011 }, faceLoops, faceKeys, tolerance);
                    }
                }
            }
        }

        private static IReadOnlyList<Point3d> CollectUniqueNodes(
            IReadOnlyList<LayerData> layers,
            IReadOnlyList<Line> voxelEdges,
            double tolerance)
        {
            var nodes = new List<Point3d>();

            foreach (LayerData layer in layers)
            {
                foreach (List<Point3d> ring in layer.Rings)
                {
                    foreach (Point3d point in ring)
                    {
                        TryAddPoint(point, nodes, tolerance);
                    }
                }
            }

            foreach (Line edge in voxelEdges)
            {
                TryAddPoint(edge.From, nodes, tolerance);
                TryAddPoint(edge.To, nodes, tolerance);
            }

            return nodes;
        }

        private static string BuildAnalysis(
            IReadOnlyList<LayerData> layers,
            int sharedPerimeterDivisions,
            int sharedRadialBands,
            int nodeCount,
            int edgeCount,
            int voxelCount,
            int faceCount,
            IReadOnlyList<string> notes)
        {
            double averagePerimeter = layers.Count == 0 ? 0.0 : layers.Average(layer => ComputeClosedLength(layer.PerimeterPoints));
            double averagePerimeterSpacing = sharedPerimeterDivisions <= 0 ? 0.0 : averagePerimeter / sharedPerimeterDivisions;

            var lines = new List<string>
            {
                "Layers: " + layers.Count,
                "Shared perimeter divisions: " + sharedPerimeterDivisions,
                "Shared radial bands: " + sharedRadialBands,
                "Average perimeter spacing: " + averagePerimeterSpacing.ToString("0.###"),
                "Nodes: " + nodeCount,
                "Edges: " + edgeCount,
                "Voxel cells: " + voxelCount,
                "Face loops: " + faceCount,
                "Ring order: layer-major, outer ring first and inner rings following per layer."
            };

            if (notes.Count > 0)
            {
                lines.Add("Notes: " + string.Join(" | ", notes.Distinct()));
            }

            return string.Join(Environment.NewLine, lines);
        }

        private static List<Point3d> ExtractClosedLayerPoints(Curve curve, double fallbackSegmentLength, double tolerance)
        {
            var points = new List<Point3d>();

            if (curve.TryGetPolyline(out Polyline polyline) && polyline.Count >= 4)
            {
                points.AddRange(polyline);
            }
            else
            {
                double safeSegmentLength = Math.Max(fallbackSegmentLength, tolerance * 4.0);
                double curveLength = curve.GetLength();
                int sampleCount = Math.Max(3, (int)Math.Round(curveLength / safeSegmentLength));

                for (int index = 0; index < sampleCount; index++)
                {
                    double normalizedLength = (double)index / sampleCount;
                    if (!curve.NormalizedLengthParameter(normalizedLength, out double parameter))
                    {
                        parameter = curve.Domain.ParameterAt(normalizedLength);
                    }

                    points.Add(curve.PointAt(parameter));
                }
            }

            if (points.Count > 1 && points[0].DistanceTo(points[points.Count - 1]) <= tolerance)
            {
                points.RemoveAt(points.Count - 1);
            }

            return RemoveSequentialDuplicates(points, tolerance);
        }

        private static Plane ResolveLayerPlane(IReadOnlyList<Point3d> points, Plane? suppliedPlane)
        {
            if (suppliedPlane.HasValue && suppliedPlane.Value.IsValid)
            {
                return suppliedPlane.Value;
            }

            Plane fitPlane;
            if (Plane.FitPlaneToPoints(points, out fitPlane) == PlaneFitResult.Success && fitPlane.IsValid)
            {
                return fitPlane;
            }

            return new Plane(AveragePoint(points), Vector3d.ZAxis);
        }

        private static double ResolveLayerSeamParameter(LayerData layer, Curve? seamGuideCurve, double tolerance)
        {
            Curve perimeterCurve = CreateClosedPolylineCurve(layer.PerimeterPoints);
            if (seamGuideCurve != null && seamGuideCurve.IsValid)
            {
                double guideParameter = FindGuideIntersectionParameter(seamGuideCurve, layer.Plane, tolerance);
                Point3d guidePoint = seamGuideCurve.PointAt(guideParameter);
                return FindCurveNormalizedLengthAtClosestPoint(perimeterCurve, guidePoint);
            }

            Point3d centroid = AveragePoint(layer.PerimeterPoints);
            int bestIndex = 0;
            double bestScore = double.MinValue;

            for (int index = 0; index < layer.PerimeterPoints.Count; index++)
            {
                Vector3d direction = layer.PerimeterPoints[index] - centroid;
                double score = direction * layer.Plane.XAxis;
                if (score > bestScore)
                {
                    bestScore = score;
                    bestIndex = index;
                }
            }

            return (double)bestIndex / layer.PerimeterPoints.Count;
        }

        private static Vector3d ResolveGuideDirection(Plane plane, Curve? guideCurve, double tolerance)
        {
            if (guideCurve == null || !guideCurve.IsValid)
            {
                return Vector3d.Unset;
            }

            double parameter = FindGuideIntersectionParameter(guideCurve, plane, tolerance);
            Vector3d tangent = guideCurve.TangentAt(parameter);
            if (!tangent.Unitize())
            {
                return Vector3d.Unset;
            }

            Vector3d projected = tangent - ((tangent * plane.Normal) * plane.Normal);
            if (!projected.Unitize())
            {
                return Vector3d.Unset;
            }

            return projected;
        }

        private static double ComputeSecondPointVerticalityScore(LayerData previousLayer, LayerData currentLayer, bool reverseCurrent)
        {
            IReadOnlyList<Point3d> previousRing = previousLayer.PerimeterPoints;
            IReadOnlyList<Point3d> currentRing = reverseCurrent
                ? CreateReversePreservingStartCopy(currentLayer.PerimeterPoints)
                : currentLayer.PerimeterPoints;

            int count = Math.Min(previousRing.Count, currentRing.Count);
            if (count < 3)
            {
                return double.MaxValue;
            }

            int lastIndex = count - 1;
            return ComputePlanarDeviation(previousRing[1], currentRing[1]) +
                   ComputePlanarDeviation(previousRing[lastIndex], currentRing[lastIndex]);
        }

        private static double ComputePlanarDeviation(Point3d a, Point3d b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;
            return Math.Sqrt((dx * dx) + (dy * dy));
        }

        private static double EstimateLayerUsableRadius(LayerData layer, double tolerance)
        {
            layer.RebuildProjectedData();
            double minRadius = double.MaxValue;
            foreach (Point3d point in layer.ProjectedPerimeter)
            {
                minRadius = Math.Min(minRadius, layer.CenterOnPlane.DistanceTo(point));
            }

            return minRadius <= tolerance ? 0.0 : minRadius * 0.9;
        }

        private static Curve? TryCreateInwardOffset(Curve perimeterCurve, Plane plane, double offsetDistance, double tolerance)
        {
            AreaMassProperties? sourceAreaProperties = AreaMassProperties.Compute(perimeterCurve);
            if (sourceAreaProperties == null)
            {
                return null;
            }

            double sourceArea = Math.Abs(sourceAreaProperties.Area);
            Curve[]? positiveOffsets = perimeterCurve.Offset(plane, offsetDistance, tolerance, CurveOffsetCornerStyle.Sharp);
            Curve[]? negativeOffsets = perimeterCurve.Offset(plane, -offsetDistance, tolerance, CurveOffsetCornerStyle.Sharp);

            Curve? bestCurve = null;
            double bestArea = double.MinValue;

            foreach (Curve candidate in (positiveOffsets ?? Array.Empty<Curve>()).Concat(negativeOffsets ?? Array.Empty<Curve>()))
            {
                if (candidate == null || !candidate.IsValid || !candidate.IsClosed)
                {
                    continue;
                }

                AreaMassProperties? areaProperties = AreaMassProperties.Compute(candidate);
                if (areaProperties == null)
                {
                    continue;
                }

                double area = Math.Abs(areaProperties.Area);
                if (area <= tolerance * tolerance || area >= sourceArea)
                {
                    continue;
                }

                if (area > bestArea)
                {
                    bestArea = area;
                    bestCurve = candidate;
                }
            }

            return bestCurve;
        }

        private static List<Point3d> BuildOffsetDrivenInteriorRing(
            LayerData layer,
            Curve inwardOffset,
            double offsetDistance,
            double tolerance)
        {
            var ring = new List<Point3d>(layer.PerimeterPoints.Count);

            for (int index = 0; index < layer.PerimeterPoints.Count; index++)
            {
                Point3d projectedPoint = layer.ProjectedPerimeter[index];
                Point3d perimeterPoint = layer.PerimeterPoints[index];
                double projectedRadius = layer.CenterOnPlane.DistanceTo(projectedPoint);

                if (projectedRadius <= tolerance)
                {
                    ring.Add(layer.CenterInSpace);
                    continue;
                }

                Vector3d direction = projectedPoint - layer.CenterOnPlane;
                Point3d farPoint = layer.CenterOnPlane + (direction * 2.5);
                CurveIntersections intersections = Intersection.CurveLine(inwardOffset, new Line(layer.CenterOnPlane, farPoint), tolerance, tolerance);
                double bestDistance = 0.0;

                if (intersections != null)
                {
                    foreach (IntersectionEvent intersection in intersections)
                    {
                        Point3d candidate = intersection.PointA;
                        Vector3d candidateDirection = candidate - layer.CenterOnPlane;
                        if ((direction * candidateDirection) < 0.0)
                        {
                            continue;
                        }

                        double distance = layer.CenterOnPlane.DistanceTo(candidate);
                        if (distance > bestDistance && distance < projectedRadius + (tolerance * 8.0))
                        {
                            bestDistance = distance;
                        }
                    }
                }

                double alpha = bestDistance > 0.0
                    ? bestDistance / projectedRadius
                    : Math.Max(0.0, (projectedRadius - offsetDistance) / projectedRadius);

                ring.Add(layer.CenterInSpace + ((perimeterPoint - layer.CenterInSpace) * Math.Max(0.0, Math.Min(1.0, alpha))));
            }

            return ring;
        }

        private static List<Point3d> BuildScaledInteriorRing(LayerData layer, double alpha, double tolerance)
        {
            double clampedAlpha = Math.Max(0.1, Math.Min(0.95, alpha));
            var ring = new List<Point3d>(layer.PerimeterPoints.Count);

            foreach (Point3d point in layer.PerimeterPoints)
            {
                ring.Add(layer.CenterInSpace + ((point - layer.CenterInSpace) * clampedAlpha));
            }

            return RemoveSequentialDuplicates(ring, tolerance);
        }

        private static bool TryAddEdge(
            Point3d from,
            Point3d to,
            ICollection<Line> edges,
            ISet<string> edgeKeys,
            double tolerance)
        {
            if (from.DistanceTo(to) <= tolerance)
            {
                return false;
            }

            string key = BuildUndirectedPointPairKey(from, to, tolerance);
            if (!edgeKeys.Add(key))
            {
                return false;
            }

            edges.Add(new Line(from, to));
            return true;
        }

        private static bool TryAddFace(
            IReadOnlyList<Point3d> points,
            ICollection<Curve> faces,
            ISet<string> faceKeys,
            double tolerance)
        {
            var cleaned = RemoveSequentialDuplicates(points.ToList(), tolerance);
            if (cleaned.Count < 3)
            {
                return false;
            }

            string key = BuildFaceKey(cleaned, tolerance);
            if (!faceKeys.Add(key))
            {
                return false;
            }

            faces.Add(CreateClosedPolylineCurve(cleaned));
            return true;
        }

        private static string BuildUndirectedPointPairKey(Point3d a, Point3d b, double tolerance)
        {
            string pointA = QuantizePoint(a, tolerance);
            string pointB = QuantizePoint(b, tolerance);
            return string.CompareOrdinal(pointA, pointB) <= 0 ? pointA + "|" + pointB : pointB + "|" + pointA;
        }

        private static string BuildFaceKey(IReadOnlyList<Point3d> points, double tolerance)
        {
            return string.Join("|", points.Select(point => QuantizePoint(point, tolerance)).OrderBy(text => text));
        }

        private static string QuantizePoint(Point3d point, double tolerance)
        {
            double scale = Math.Max(tolerance, 1e-6);
            long x = (long)Math.Round(point.X / scale);
            long y = (long)Math.Round(point.Y / scale);
            long z = (long)Math.Round(point.Z / scale);
            return x + "," + y + "," + z;
        }

        private static void TryAddPoint(Point3d point, ICollection<Point3d> points, double tolerance)
        {
            foreach (Point3d existing in points)
            {
                if (existing.DistanceTo(point) <= tolerance)
                {
                    return;
                }
            }

            points.Add(point);
        }

        private static double FindGuideIntersectionParameter(Curve guideCurve, Plane plane, double tolerance)
        {
            CurveIntersections intersections = Intersection.CurvePlane(guideCurve, plane, tolerance);
            if (intersections != null && intersections.Count > 0)
            {
                IntersectionEvent bestIntersection = intersections[0];
                double bestDistance = double.MaxValue;

                foreach (IntersectionEvent intersection in intersections)
                {
                    double distance = plane.DistanceTo(intersection.PointA);
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        bestIntersection = intersection;
                    }
                }

                return bestIntersection.ParameterA;
            }

            if (guideCurve.ClosestPoint(plane.Origin, out double closestParameter))
            {
                return closestParameter;
            }

            return guideCurve.Domain.ParameterAt(0.0);
        }

        private static double FindCurveNormalizedLengthAtClosestPoint(Curve curve, Point3d point)
        {
            if (!curve.ClosestPoint(point, out double parameter))
            {
                return 0.0;
            }

            double totalLength = curve.GetLength();
            if (totalLength <= 1e-9)
            {
                return 0.0;
            }

            Interval partialDomain = new Interval(curve.Domain.Min, parameter);
            return NormalizeUnitParameter(curve.GetLength(partialDomain) / totalLength);
        }

        private static Point3d AveragePoint(IReadOnlyList<Point3d> points)
        {
            if (points.Count == 0)
            {
                return Point3d.Origin;
            }

            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            foreach (Point3d point in points)
            {
                x += point.X;
                y += point.Y;
                z += point.Z;
            }

            double scale = 1.0 / points.Count;
            return new Point3d(x * scale, y * scale, z * scale);
        }

        private static Point3d ComputePlanarCentroid(IReadOnlyList<Point3d> projectedPoints, Plane plane)
        {
            var coordinates = new List<Point2d>(projectedPoints.Count);
            foreach (Point3d point in projectedPoints)
            {
                double u;
                double v;
                plane.ClosestParameter(point, out u, out v);
                coordinates.Add(new Point2d(u, v));
            }

            double signedAreaTwice = 0.0;
            double centroidU = 0.0;
            double centroidV = 0.0;
            for (int index = 0; index < coordinates.Count; index++)
            {
                Point2d current = coordinates[index];
                Point2d next = coordinates[(index + 1) % coordinates.Count];
                double cross = (current.X * next.Y) - (next.X * current.Y);
                signedAreaTwice += cross;
                centroidU += (current.X + next.X) * cross;
                centroidV += (current.Y + next.Y) * cross;
            }

            if (Math.Abs(signedAreaTwice) <= 1e-9)
            {
                return plane.PointAt(coordinates.Average(point => point.X), coordinates.Average(point => point.Y));
            }

            double scale = 1.0 / (3.0 * signedAreaTwice);
            return plane.PointAt(centroidU * scale, centroidV * scale);
        }

        private static List<Point3d> ProjectPointsToPlane(IReadOnlyList<Point3d> points, Plane plane)
        {
            var projected = new List<Point3d>(points.Count);
            foreach (Point3d point in points)
            {
                projected.Add(plane.ClosestPoint(point));
            }

            return projected;
        }

        private static double ComputeClosedLength(IReadOnlyList<Point3d> points)
        {
            double length = 0.0;
            for (int index = 0; index < points.Count; index++)
            {
                length += points[index].DistanceTo(points[(index + 1) % points.Count]);
            }

            return length;
        }

        private static double ComputeAverageClosedSegmentLength(IReadOnlyList<Point3d> points)
        {
            return points.Count == 0 ? 0.0 : ComputeClosedLength(points) / points.Count;
        }

        private static double NormalizeUnitParameter(double parameter)
        {
            double normalized = parameter % 1.0;
            return normalized < 0.0 ? normalized + 1.0 : normalized;
        }

        private static List<Point3d> RemoveSequentialDuplicates(IReadOnlyList<Point3d> points, double tolerance)
        {
            var cleaned = new List<Point3d>(points.Count);
            foreach (Point3d point in points)
            {
                if (cleaned.Count == 0 || cleaned[cleaned.Count - 1].DistanceTo(point) > tolerance)
                {
                    cleaned.Add(point);
                }
            }

            if (cleaned.Count > 1 && cleaned[0].DistanceTo(cleaned[cleaned.Count - 1]) <= tolerance)
            {
                cleaned.RemoveAt(cleaned.Count - 1);
            }

            return cleaned;
        }

        private static List<Point3d> CreateReversePreservingStartCopy(IReadOnlyList<Point3d> points)
        {
            if (points.Count <= 2)
            {
                return new List<Point3d>(points);
            }

            var reversed = new List<Point3d>(points.Count) { points[0] };
            for (int index = points.Count - 1; index >= 1; index--)
            {
                reversed.Add(points[index]);
            }

            return reversed;
        }

        private static PolylineCurve CreateClosedPolylineCurve(IReadOnlyList<Point3d> points)
        {
            var polyline = new Polyline(points);
            polyline.Add(points[0]);
            return new PolylineCurve(polyline);
        }

        private sealed class LayerData
        {
            public LayerData(int stackIndex, Plane plane, List<Point3d> perimeterPoints)
            {
                StackIndex = stackIndex;
                Plane = plane;
                PerimeterPoints = perimeterPoints;
                Rings = new List<List<Point3d>>();
                RingCurves = new List<Curve>();
                RebuildProjectedData();
            }

            public int StackIndex { get; }
            public Plane Plane { get; }
            public List<Point3d> PerimeterPoints { get; private set; }
            public List<Point3d> ProjectedPerimeter { get; private set; } = new List<Point3d>();
            public Point3d CenterOnPlane { get; private set; }
            public Point3d CenterInSpace { get; private set; }
            public List<List<Point3d>> Rings { get; }
            public List<Curve> RingCurves { get; }

            public void NormalizePerimeter(int divisionCount, double seamParameter)
            {
                PerimeterPoints = new TopologicalRingSampler(PerimeterPoints).Sample(divisionCount, seamParameter);
                RebuildProjectedData();
            }

            public void ReversePreservingStart()
            {
                PerimeterPoints = CreateReversePreservingStartCopy(PerimeterPoints);
                RebuildProjectedData();
            }

            public Vector3d GetStartTraversalDirection()
            {
                return new TopologicalRingSampler(PerimeterPoints).TangentAt(0.0);
            }

            public void RebuildProjectedData()
            {
                ProjectedPerimeter = ProjectPointsToPlane(PerimeterPoints, Plane);
                CenterOnPlane = ComputePlanarCentroid(ProjectedPerimeter, Plane);
                CenterInSpace = AveragePoint(PerimeterPoints);
            }
        }

        private sealed class TopologicalRingSampler
        {
            private readonly IReadOnlyList<Point3d> _points;
            private readonly IReadOnlyList<double> _vertexParameters;

            public TopologicalRingSampler(IReadOnlyList<Point3d> points)
            {
                _points = points;
                _vertexParameters = BuildVertexParameters(points);
            }

            public List<Point3d> Sample(int count, double seamParameter = 0.0)
            {
                var samples = new List<Point3d>(count);
                double normalizedSeam = NormalizeUnitParameter(seamParameter);
                for (int index = 0; index < count; index++)
                {
                    samples.Add(Evaluate(normalizedSeam + ((double)index / count)));
                }

                return samples;
            }

            public Point3d Evaluate(double normalizedParameter)
            {
                if (_points.Count == 0)
                {
                    return Point3d.Origin;
                }

                if (_points.Count == 1)
                {
                    return _points[0];
                }

                double parameter = NormalizeUnitParameter(normalizedParameter);
                int startIndex = _points.Count - 1;
                for (int index = 0; index < _vertexParameters.Count; index++)
                {
                    double start = _vertexParameters[index];
                    double end = index == _vertexParameters.Count - 1 ? 1.0 : _vertexParameters[index + 1];
                    if (parameter >= start && parameter <= end + 1e-9)
                    {
                        startIndex = index;
                        break;
                    }
                }

                int endIndex = (startIndex + 1) % _points.Count;
                double startParameter = _vertexParameters[startIndex];
                double endParameter = startIndex == _vertexParameters.Count - 1 ? 1.0 : _vertexParameters[endIndex];
                double local = (parameter - startParameter) / Math.Max(1e-9, endParameter - startParameter);
                return _points[startIndex] + ((_points[endIndex] - _points[startIndex]) * local);
            }

            public Vector3d TangentAt(double normalizedParameter)
            {
                if (_points.Count < 2)
                {
                    return Vector3d.Unset;
                }

                double parameter = NormalizeUnitParameter(normalizedParameter);
                int startIndex = _points.Count - 1;
                for (int index = 0; index < _vertexParameters.Count; index++)
                {
                    double start = _vertexParameters[index];
                    double end = index == _vertexParameters.Count - 1 ? 1.0 : _vertexParameters[index + 1];
                    if (parameter >= start && parameter <= end + 1e-9)
                    {
                        startIndex = index;
                        break;
                    }
                }

                Vector3d tangent = _points[(startIndex + 1) % _points.Count] - _points[startIndex];
                return tangent.Unitize() ? tangent : Vector3d.Unset;
            }

            private static IReadOnlyList<double> BuildVertexParameters(IReadOnlyList<Point3d> points)
            {
                if (points.Count == 0)
                {
                    return Array.Empty<double>();
                }

                if (points.Count == 1)
                {
                    return new[] { 0.0 };
                }

                var segmentLengths = new double[points.Count];
                double totalLength = 0.0;
                for (int index = 0; index < points.Count; index++)
                {
                    double length = points[index].DistanceTo(points[(index + 1) % points.Count]);
                    segmentLengths[index] = length;
                    totalLength += length;
                }

                if (totalLength <= 1e-9)
                {
                    return Enumerable.Range(0, points.Count).Select(index => (double)index / points.Count).ToArray();
                }

                var parameters = new double[points.Count];
                double cumulative = 0.0;
                for (int index = 1; index < points.Count; index++)
                {
                    cumulative += segmentLengths[index - 1];
                    parameters[index] = cumulative / totalLength;
                }

                return parameters;
            }
        }
    }
}
