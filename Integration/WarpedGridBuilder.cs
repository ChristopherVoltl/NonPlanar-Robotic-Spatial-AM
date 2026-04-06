using System;
using System.Collections.Generic;
using System.Linq;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Controls warped-grid generation from ordered nonplanar perimeter layers.
    /// </summary>
    internal sealed class WarpedGridOptions
    {
        /// <summary>
        /// Gets or sets the fallback segment length used when a layer curve is not already a usable polyline.
        /// </summary>
        public double FallbackSegmentLength { get; set; } = 10.0;

        /// <summary>
        /// Gets or sets the number of quads per side of the base square grid.
        /// </summary>
        /// <remarks>
        /// Preconditions: should be greater than zero. Values below one are clamped internally.
        /// </remarks>
        public int GridResolution { get; set; } = 8;

        /// <summary>
        /// Gets or sets how strongly boundary curvature pulls grid samples toward tighter features.
        /// </summary>
        /// <remarks>
        /// Preconditions: should be between 0 and 1. Values are clamped internally.
        /// </remarks>
        public double AdaptiveStrength { get; set; } = 0.0;

        /// <summary>
        /// Gets or sets how many smoothing passes are applied to the boundary refinement weights.
        /// </summary>
        /// <remarks>
        /// Preconditions: should be zero or greater. Larger values make the refinement field smoother but less local.
        /// </remarks>
        public int AdaptiveSmoothingPasses { get; set; } = 1;

        /// <summary>
        /// Gets or sets how many recursive refinement passes can insert extra grid lines.
        /// </summary>
        /// <remarks>
        /// Preconditions: should be zero or greater. A value of zero disables true subdivision and keeps only the base grid.
        /// </remarks>
        public int AdaptiveLevels { get; set; } = 2;

        /// <summary>
        /// Gets or sets the geometric deviation threshold that must be exceeded before an interval is subdivided.
        /// </summary>
        /// <remarks>
        /// Preconditions: should be greater than zero. Intervals whose boundary approximation error stays below this
        /// threshold keep their current resolution.
        /// </remarks>
        public double AdaptiveDeviationThreshold { get; set; } = 1.0;

        /// <summary>
        /// Gets or sets an optional curve used to anchor the seam for every normalized perimeter.
        /// </summary>
        public Curve? StartGuideCurve { get; set; }

        /// <summary>
        /// Gets or sets an optional curve used to bias the traversal direction after the seam is fixed.
        /// </summary>
        public Curve? DirectionGuideCurve { get; set; }
    }

    /// <summary>
    /// Bundles the warped-grid geometry created from the layer stack.
    /// </summary>
    internal sealed class WarpedGridResult
    {
        public WarpedGridResult(
            IReadOnlyList<Mesh> layerMeshes,
            IReadOnlyList<Curve> gridCurves,
            IReadOnlyList<Line> interLayerStruts,
            IReadOnlyList<Point3d> nodes,
            IReadOnlyList<Point3d> cellCenters,
            int boundaryDivisionCount,
            string analysis)
        {
            LayerMeshes = layerMeshes;
            GridCurves = gridCurves;
            InterLayerStruts = interLayerStruts;
            Nodes = nodes;
            CellCenters = cellCenters;
            BoundaryDivisionCount = boundaryDivisionCount;
            Analysis = analysis;
        }

        public IReadOnlyList<Mesh> LayerMeshes { get; }
        public IReadOnlyList<Curve> GridCurves { get; }
        public IReadOnlyList<Line> InterLayerStruts { get; }
        public IReadOnlyList<Point3d> Nodes { get; }
        public IReadOnlyList<Point3d> CellCenters { get; }
        public int BoundaryDivisionCount { get; }
        public string Analysis { get; }
    }

    /// <summary>
    /// Builds a square warped-grid scaffold that morphs to each nonplanar perimeter layer.
    /// </summary>
    /// <remarks>
    /// The builder starts from a square parameter domain and morphs that grid to each normalized layer boundary
    /// using a Coons-style interpolation. This differs from <see cref="ConformalVoxelBuilder"/> because the primary
    /// artifact is a quad mesh per layer, not offset ring bands. Input order defines the layer stack order.
    /// </remarks>
    internal static class WarpedGridBuilder
    {
        public static WarpedGridResult Build(
            IReadOnlyList<Curve> layerCurves,
            IReadOnlyList<Plane> layerPlanes,
            WarpedGridOptions options,
            double tolerance)
        {
            if (options == null)
            {
                throw new ArgumentNullException(nameof(options));
            }

            var layerMeshes = new List<Mesh>();
            var gridCurves = new List<Curve>();
            var interLayerStruts = new List<Line>();
            var nodes = new List<Point3d>();
            var cellCenters = new List<Point3d>();
            var notes = new List<string>();

            if (layerCurves == null || layerCurves.Count == 0)
            {
                return new WarpedGridResult(
                    layerMeshes,
                    gridCurves,
                    interLayerStruts,
                    nodes,
                    cellCenters,
                    0,
                    "No layer curves were supplied. Input order defines stack order for this component.");
            }

            double safeTolerance = Math.Max(tolerance, 1e-6);
            int gridResolution = Math.Max(1, options.GridResolution);
            double adaptiveStrength = Math.Max(0.0, Math.Min(1.0, options.AdaptiveStrength));
            int adaptiveSmoothingPasses = Math.Max(0, options.AdaptiveSmoothingPasses);
            int adaptiveLevels = Math.Max(0, options.AdaptiveLevels);
            double adaptiveDeviationThreshold = Math.Max(safeTolerance, options.AdaptiveDeviationThreshold);
            int boundaryDivisionCount = Math.Max(4, gridResolution * 4);
            int denseBoundaryDivisionCount = Math.Max(boundaryDivisionCount * 4, 16);

            var layers = new List<LayerData>();
            for (int index = 0; index < layerCurves.Count; index++)
            {
                Plane? plane = index < layerPlanes.Count ? layerPlanes[index] : (Plane?)null;
                LayerData? layer = TryBuildLayer(index, layerCurves[index], plane, options.FallbackSegmentLength, safeTolerance, notes);
                if (layer != null)
                {
                    layers.Add(layer);
                }
            }

            if (layers.Count == 0)
            {
                return new WarpedGridResult(
                    layerMeshes,
                    gridCurves,
                    interLayerStruts,
                    nodes,
                    cellCenters,
                    0,
                    "No valid closed layers were available for warped-grid generation.");
            }

            NormalizeLayerStack(layers, boundaryDivisionCount, denseBoundaryDivisionCount, options.StartGuideCurve, options.DirectionGuideCurve, safeTolerance, notes);

            foreach (LayerData layer in layers)
            {
                IReadOnlyList<double> uParameters = BuildAdaptiveParameterSet(
                    layer,
                    denseBoundaryDivisionCount / 4,
                    gridResolution,
                    adaptiveStrength,
                    adaptiveSmoothingPasses,
                    adaptiveLevels,
                    adaptiveDeviationThreshold,
                    useHorizontalSides: true);

                IReadOnlyList<double> vParameters = BuildAdaptiveParameterSet(
                    layer,
                    denseBoundaryDivisionCount / 4,
                    gridResolution,
                    adaptiveStrength,
                    adaptiveSmoothingPasses,
                    adaptiveLevels,
                    adaptiveDeviationThreshold,
                    useHorizontalSides: false);

                BuildLayerMesh(layer, uParameters, vParameters, gridCurves);
                layerMeshes.Add(layer.Mesh!);
                foreach (Point3d point in layer.GridPoints!)
                {
                    TryAddPoint(point, nodes, safeTolerance);
                }
            }

            BuildInterLayerGeometry(layers, interLayerStruts, cellCenters, safeTolerance);
            foreach (Line strut in interLayerStruts)
            {
                TryAddPoint(strut.From, nodes, safeTolerance);
                TryAddPoint(strut.To, nodes, safeTolerance);
            }

            int maxUResolution = layers.Count == 0 ? 0 : layers.Max(layer => Math.Max(0, layer.UCount - 1));
            int maxVResolution = layers.Count == 0 ? 0 : layers.Max(layer => Math.Max(0, layer.VCount - 1));
            string analysis = BuildAnalysis(layers, gridResolution, maxUResolution, maxVResolution, boundaryDivisionCount, adaptiveStrength, adaptiveLevels, adaptiveDeviationThreshold, interLayerStruts.Count, cellCenters.Count, notes);
            return new WarpedGridResult(layerMeshes, gridCurves, interLayerStruts, nodes, cellCenters, boundaryDivisionCount, analysis);
        }

        private static LayerData? TryBuildLayer(
            int stackIndex,
            Curve curve,
            Plane? suppliedPlane,
            double fallbackSegmentLength,
            double tolerance,
            ICollection<string> notes)
        {
            if (curve == null || !curve.IsValid)
            {
                notes.Add("Skipped an invalid layer curve.");
                return null;
            }

            List<Point3d> perimeterPoints = ExtractClosedLayerPoints(curve, fallbackSegmentLength, tolerance);
            if (perimeterPoints.Count < 3)
            {
                notes.Add("Skipped a layer because it was not a usable closed loop.");
                return null;
            }

            Plane plane = ResolveLayerPlane(perimeterPoints, suppliedPlane);
            return new LayerData(stackIndex, plane, perimeterPoints);
        }

        private static void NormalizeLayerStack(
            IReadOnlyList<LayerData> layers,
            int boundaryDivisionCount,
            int denseBoundaryDivisionCount,
            Curve? startGuideCurve,
            Curve? directionGuideCurve,
            double tolerance,
            ICollection<string> notes)
        {
            foreach (LayerData layer in layers)
            {
                double seamParameter = ResolveLayerSeamParameter(layer, startGuideCurve, tolerance);
                layer.NormalizePerimeter(boundaryDivisionCount, denseBoundaryDivisionCount, seamParameter);
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

        private static void BuildLayerMesh(
            LayerData layer,
            IReadOnlyList<double> uParameters,
            IReadOnlyList<double> vParameters,
            ICollection<Curve> gridCurves)
        {
            int uCount = Math.Max(2, uParameters.Count);
            int vCount = Math.Max(2, vParameters.Count);

            Point3d[,] grid = new Point3d[uCount, vCount];

            for (int vIndex = 0; vIndex < vCount; vIndex++)
            {
                double v = vParameters[vIndex];
                for (int uIndex = 0; uIndex < uCount; uIndex++)
                {
                    double u = uParameters[uIndex];
                    grid[uIndex, vIndex] = EvaluateLayerPoint(layer, u, v);
                }
            }

            var mesh = new Mesh();
            for (int vIndex = 0; vIndex < vCount; vIndex++)
            {
                for (int uIndex = 0; uIndex < uCount; uIndex++)
                {
                    mesh.Vertices.Add(grid[uIndex, vIndex]);
                }
            }

            for (int vIndex = 0; vIndex < vCount - 1; vIndex++)
            {
                for (int uIndex = 0; uIndex < uCount - 1; uIndex++)
                {
                    int a = GridIndex(uIndex, vIndex, uCount);
                    int b = GridIndex(uIndex + 1, vIndex, uCount);
                    int c = GridIndex(uIndex + 1, vIndex + 1, uCount);
                    int d = GridIndex(uIndex, vIndex + 1, uCount);
                    mesh.Faces.AddFace(a, b, c, d);
                }
            }

            mesh.Normals.ComputeNormals();
            mesh.Compact();

            layer.Mesh = mesh;
            layer.UParameters = uParameters.ToArray();
            layer.VParameters = vParameters.ToArray();
            layer.UCount = uCount;
            layer.VCount = vCount;
            layer.GridPoints = FlattenGrid(grid, uCount, vCount);

            for (int vIndex = 0; vIndex < vCount; vIndex++)
            {
                var row = new List<Point3d>(uCount);
                for (int uIndex = 0; uIndex < uCount; uIndex++)
                {
                    row.Add(grid[uIndex, vIndex]);
                }

                gridCurves.Add(CreateOpenPolylineCurve(row));
            }

            for (int uIndex = 0; uIndex < uCount; uIndex++)
            {
                var column = new List<Point3d>(vCount);
                for (int vIndex = 0; vIndex < vCount; vIndex++)
                {
                    column.Add(grid[uIndex, vIndex]);
                }

                gridCurves.Add(CreateOpenPolylineCurve(column));
            }
        }

        private static void BuildInterLayerGeometry(
            IReadOnlyList<LayerData> layers,
            ICollection<Line> interLayerStruts,
            ICollection<Point3d> cellCenters,
            double tolerance)
        {
            var edgeKeys = new HashSet<string>(StringComparer.Ordinal);

            for (int layerIndex = 0; layerIndex < layers.Count - 1; layerIndex++)
            {
                LayerData lower = layers[layerIndex];
                LayerData upper = layers[layerIndex + 1];
                if (lower.Mesh == null || upper.Mesh == null || lower.GridPoints == null || upper.GridPoints == null)
                {
                    continue;
                }

                IReadOnlyList<double> mergedU = MergeParameterSets(lower.UParameters, upper.UParameters);
                IReadOnlyList<double> mergedV = MergeParameterSets(lower.VParameters, upper.VParameters);

                foreach (double v in mergedV)
                {
                    foreach (double u in mergedU)
                    {
                        TryAddEdge(EvaluateLayerPoint(lower, u, v), EvaluateLayerPoint(upper, u, v), interLayerStruts, edgeKeys, tolerance);
                    }
                }

                for (int vIndex = 0; vIndex < mergedV.Count - 1; vIndex++)
                {
                    for (int uIndex = 0; uIndex < mergedU.Count - 1; uIndex++)
                    {
                        double u0 = mergedU[uIndex];
                        double u1 = mergedU[uIndex + 1];
                        double v0 = mergedV[vIndex];
                        double v1 = mergedV[vIndex + 1];

                        Point3d p000 = EvaluateLayerPoint(lower, u0, v0);
                        Point3d p100 = EvaluateLayerPoint(lower, u1, v0);
                        Point3d p110 = EvaluateLayerPoint(lower, u1, v1);
                        Point3d p010 = EvaluateLayerPoint(lower, u0, v1);
                        Point3d p001 = EvaluateLayerPoint(upper, u0, v0);
                        Point3d p101 = EvaluateLayerPoint(upper, u1, v0);
                        Point3d p111 = EvaluateLayerPoint(upper, u1, v1);
                        Point3d p011 = EvaluateLayerPoint(upper, u0, v1);
                        cellCenters.Add(AveragePoint(new[] { p000, p100, p110, p010, p001, p101, p111, p011 }));
                    }
                }
            }
        }

        private static IReadOnlyList<double> BuildAdaptiveParameterSet(
            LayerData layer,
            int denseSideCount,
            int gridResolution,
            double adaptiveStrength,
            int adaptiveSmoothingPasses,
            int adaptiveLevels,
            double adaptiveDeviationThreshold,
            bool useHorizontalSides)
        {
            if ((adaptiveStrength <= 1e-6 && adaptiveLevels <= 0))
            {
                return Enumerable.Range(0, gridResolution + 1).Select(index => (double)index / gridResolution).ToArray();
            }

            List<Point3d> bottomDense = ExtractSidePoints(layer.DensePerimeterPoints, 0, denseSideCount);
            List<Point3d> rightDense = ExtractSidePoints(layer.DensePerimeterPoints, denseSideCount, denseSideCount);
            List<Point3d> topDense = ExtractSidePoints(layer.DensePerimeterPoints, denseSideCount * 2, denseSideCount);
            List<Point3d> leftDense = ExtractSidePoints(layer.DensePerimeterPoints, denseSideCount * 3, denseSideCount);

            var sideSamples = new List<List<Point3d>>();
            if (useHorizontalSides)
            {
                topDense.Reverse();
                sideSamples.Add(bottomDense);
                sideSamples.Add(topDense);
            }
            else
            {
                leftDense.Reverse();
                sideSamples.Add(rightDense);
                sideSamples.Add(leftDense);
            }

            if (sideSamples.Count == 0)
            {
                return Enumerable.Range(0, gridResolution + 1).Select(index => (double)index / gridResolution).ToArray();
            }

            List<double> parameters = BuildAdaptiveSubdivisionParametersByDeviation(
                sideSamples,
                denseSideCount,
                gridResolution,
                adaptiveLevels,
                adaptiveDeviationThreshold);

            if (adaptiveStrength <= 1e-6)
            {
                return parameters.ToArray();
            }

            double[] aggregatedWeights = new double[denseSideCount + 1];
            double[] aggregatedLengths = new double[denseSideCount];
            for (int sideIndex = 0; sideIndex < sideSamples.Count; sideIndex++)
            {
                List<Point3d> side = sideSamples[sideIndex];
                double[] weights = ComputeBoundaryVertexWeights(side, adaptiveStrength);
                for (int index = 0; index < weights.Length; index++)
                {
                    aggregatedWeights[index] += weights[index];
                }

                for (int index = 0; index < denseSideCount; index++)
                {
                    aggregatedLengths[index] += side[index].DistanceTo(side[index + 1]);
                }
            }

            for (int index = 0; index < aggregatedWeights.Length; index++)
            {
                aggregatedWeights[index] /= sideSamples.Count;
            }

            for (int index = 0; index < aggregatedLengths.Length; index++)
            {
                aggregatedLengths[index] /= sideSamples.Count;
            }

            SmoothWeights(aggregatedWeights, adaptiveSmoothingPasses);
            return RemapParametersByWeightedLength(parameters, aggregatedLengths, aggregatedWeights);
        }

        private static List<Point3d> ExtractSidePoints(IReadOnlyList<Point3d> perimeterPoints, int startIndex, int segmentCount)
        {
            var points = new List<Point3d>(segmentCount + 1);
            int count = perimeterPoints.Count;
            for (int index = 0; index <= segmentCount; index++)
            {
                points.Add(perimeterPoints[(startIndex + index) % count]);
            }

            return points;
        }

        private static List<Point3d> SampleSideByParameters(IReadOnlyList<Point3d> denseSidePoints, IReadOnlyList<double> parameters)
        {
            if (denseSidePoints.Count == 0)
            {
                return new List<Point3d>();
            }

            if (denseSidePoints.Count == 1)
            {
                return parameters.Select(_ => denseSidePoints[0]).ToList();
            }

            var cumulative = new double[denseSidePoints.Count];
            double total = 0.0;
            cumulative[0] = 0.0;
            for (int index = 1; index < denseSidePoints.Count; index++)
            {
                total += denseSidePoints[index - 1].DistanceTo(denseSidePoints[index]);
                cumulative[index] = total;
            }

            if (total <= 1e-9)
            {
                return new List<Point3d>(denseSidePoints);
            }

            var samples = new List<Point3d>(parameters.Count);
            foreach (double parameter in parameters)
            {
                double target = Math.Max(0.0, Math.Min(1.0, parameter)) * total;
                int segmentIndex = 0;
                while (segmentIndex < cumulative.Length - 2 && cumulative[segmentIndex + 1] < target)
                {
                    segmentIndex++;
                }

                double start = cumulative[segmentIndex];
                double end = cumulative[segmentIndex + 1];
                double local = end - start <= 1e-9 ? 0.0 : (target - start) / (end - start);
                samples.Add(denseSidePoints[segmentIndex] + ((denseSidePoints[segmentIndex + 1] - denseSidePoints[segmentIndex]) * local));
            }

            return samples;
        }

        private static double[] ComputeBoundaryVertexWeights(IReadOnlyList<Point3d> points, double adaptiveStrength)
        {
            var weights = new double[points.Count];
            weights[0] = 1.0;
            weights[points.Count - 1] = 1.0;

            for (int index = 1; index < points.Count - 1; index++)
            {
                Vector3d incoming = points[index] - points[index - 1];
                Vector3d outgoing = points[index + 1] - points[index];
                if (!incoming.Unitize() || !outgoing.Unitize())
                {
                    weights[index] = 1.0;
                    continue;
                }

                double angle = Vector3d.VectorAngle(incoming, outgoing);
                double turnStrength = Math.Max(0.0, Math.Min(1.0, angle / Math.PI));
                weights[index] = 1.0 + (adaptiveStrength * turnStrength * 3.0);
            }

            return weights;
        }

        private static void SmoothWeights(double[] weights, int smoothingPasses)
        {
            if (weights.Length <= 2 || smoothingPasses <= 0)
            {
                return;
            }

            var buffer = new double[weights.Length];
            for (int pass = 0; pass < smoothingPasses; pass++)
            {
                buffer[0] = weights[0];
                buffer[weights.Length - 1] = weights[weights.Length - 1];
                for (int index = 1; index < weights.Length - 1; index++)
                {
                    buffer[index] = (weights[index - 1] + (weights[index] * 2.0) + weights[index + 1]) * 0.25;
                }

                Array.Copy(buffer, weights, weights.Length);
            }
        }

        private static double[] BuildNormalizedFeatureSignal(IReadOnlyList<double> weights)
        {
            var signal = new double[weights.Count];
            double maxValue = 0.0;
            for (int index = 0; index < weights.Count; index++)
            {
                signal[index] = Math.Max(0.0, weights[index] - 1.0);
                maxValue = Math.Max(maxValue, signal[index]);
            }

            if (maxValue <= 1e-9)
            {
                return signal;
            }

            for (int index = 0; index < signal.Length; index++)
            {
                signal[index] /= maxValue;
            }

            return signal;
        }

        private static List<double> BuildAdaptiveSubdivisionParametersByDeviation(
            IReadOnlyList<List<Point3d>> sideSamples,
            int denseSideCount,
            int gridResolution,
            int adaptiveLevels,
            double adaptiveDeviationThreshold)
        {
            var parameters = Enumerable.Range(0, gridResolution + 1)
                .Select(index => (double)index / gridResolution)
                .ToList();

            if (adaptiveLevels <= 0 || sideSamples.Count == 0)
            {
                return parameters;
            }

            double minimumSpan = 1.0 / Math.Pow(2.0, adaptiveLevels + 2);

            for (int level = 0; level < adaptiveLevels; level++)
            {
                var refined = new List<double> { parameters[0] };
                for (int index = 0; index < parameters.Count - 1; index++)
                {
                    double start = parameters[index];
                    double end = parameters[index + 1];
                    double intervalDeviation = EvaluateIntervalDeviation(sideSamples, denseSideCount, start, end);
                    if (intervalDeviation > adaptiveDeviationThreshold && (end - start) > minimumSpan)
                    {
                        refined.Add(0.5 * (start + end));
                    }

                    refined.Add(end);
                }

                parameters = refined
                    .Distinct()
                    .OrderBy(value => value)
                    .ToList();
            }

            return parameters;
        }

        private static double EvaluateIntervalDeviation(
            IReadOnlyList<List<Point3d>> sideSamples,
            int denseSideCount,
            double start,
            double end)
        {
            double maxDeviation = 0.0;
            foreach (List<Point3d> side in sideSamples)
            {
                if (side.Count < 2)
                {
                    continue;
                }

                Point3d startPoint = EvaluateDenseSideAt(side, start);
                Point3d endPoint = EvaluateDenseSideAt(side, end);
                if (startPoint.DistanceTo(endPoint) <= 1e-9)
                {
                    continue;
                }

                var chord = new Line(startPoint, endPoint);
                int startIndex = Math.Max(0, Math.Min(denseSideCount, (int)Math.Floor(start * denseSideCount)));
                int endIndex = Math.Max(0, Math.Min(denseSideCount, (int)Math.Ceiling(end * denseSideCount)));
                for (int index = startIndex; index <= endIndex; index++)
                {
                    Point3d samplePoint = side[index];
                    maxDeviation = Math.Max(maxDeviation, chord.DistanceTo(samplePoint, true));
                }
            }

            return maxDeviation;
        }

        private static Point3d EvaluateDenseSideAt(IReadOnlyList<Point3d> side, double parameter)
        {
            if (side.Count == 0)
            {
                return Point3d.Origin;
            }

            if (side.Count == 1)
            {
                return side[0];
            }

            double clamped = Math.Max(0.0, Math.Min(1.0, parameter));
            double scaled = clamped * (side.Count - 1);
            int startIndex = Math.Min(side.Count - 2, Math.Max(0, (int)Math.Floor(scaled)));
            double local = scaled - startIndex;
            return side[startIndex] + ((side[startIndex + 1] - side[startIndex]) * local);
        }

        private static IReadOnlyList<double> RemapParametersByWeightedLength(
            IReadOnlyList<double> parameters,
            IReadOnlyList<double> segmentLengths,
            IReadOnlyList<double> vertexWeights)
        {
            if (parameters.Count == 0)
            {
                return Array.Empty<double>();
            }

            if (parameters.Count == 1)
            {
                return new[] { 0.0 };
            }

            var cumulative = new double[segmentLengths.Count + 1];
            double total = 0.0;
            cumulative[0] = 0.0;

            for (int index = 1; index < cumulative.Length; index++)
            {
                double segmentWeight = 0.5 * (vertexWeights[index - 1] + vertexWeights[index]);
                total += segmentLengths[index - 1] * Math.Max(1e-6, segmentWeight);
                cumulative[index] = total;
            }

            if (total <= 1e-9)
            {
                return parameters.ToArray();
            }

            var remapped = new double[parameters.Count];
            for (int sampleIndex = 0; sampleIndex < parameters.Count; sampleIndex++)
            {
                double target = total * Math.Max(0.0, Math.Min(1.0, parameters[sampleIndex]));
                int segmentIndex = 0;
                while (segmentIndex < cumulative.Length - 2 && cumulative[segmentIndex + 1] < target)
                {
                    segmentIndex++;
                }

                double start = cumulative[segmentIndex];
                double end = cumulative[segmentIndex + 1];
                double local = end - start <= 1e-9 ? 0.0 : (target - start) / (end - start);
                remapped[sampleIndex] = (segmentIndex + local) / Math.Max(1.0, segmentLengths.Count);
            }

            remapped[0] = 0.0;
            remapped[remapped.Length - 1] = 1.0;
            return remapped;
        }

        private static IReadOnlyList<double> BuildWeightedParameterSamples(
            IReadOnlyList<double> segmentLengths,
            IReadOnlyList<double> vertexWeights,
            int gridResolution)
        {
            var cumulative = new double[segmentLengths.Count + 1];
            double total = 0.0;
            cumulative[0] = 0.0;

            for (int index = 1; index < cumulative.Length; index++)
            {
                double segmentWeight = 0.5 * (vertexWeights[index - 1] + vertexWeights[index]);
                total += segmentLengths[index - 1] * Math.Max(1e-6, segmentWeight);
                cumulative[index] = total;
            }

            if (total <= 1e-9)
            {
                return Enumerable.Range(0, gridResolution + 1).Select(index => (double)index / gridResolution).ToArray();
            }

            var result = new double[gridResolution + 1];
            for (int sampleIndex = 0; sampleIndex <= gridResolution; sampleIndex++)
            {
                double target = total * sampleIndex / gridResolution;
                int segmentIndex = 0;
                while (segmentIndex < cumulative.Length - 2 && cumulative[segmentIndex + 1] < target)
                {
                    segmentIndex++;
                }

                double start = cumulative[segmentIndex];
                double end = cumulative[segmentIndex + 1];
                double local = end - start <= 1e-9 ? 0.0 : (target - start) / (end - start);
                result[sampleIndex] = (segmentIndex + local) / Math.Max(1.0, segmentLengths.Count);
            }

            return result;
        }

        private static Point3d EvaluateCoonsPatch(
            Point3d bottom,
            Point3d top,
            Point3d left,
            Point3d right,
            Point3d c00,
            Point3d c10,
            Point3d c01,
            Point3d c11,
            double u,
            double v)
        {
            Point3d l = left;
            Point3d r = right;
            Point3d b = bottom;
            Point3d t = top;

            Point3d bilinear =
                (c00 * ((1.0 - u) * (1.0 - v))) +
                (c10 * (u * (1.0 - v))) +
                (c01 * ((1.0 - u) * v)) +
                (c11 * (u * v));

            Vector3d blend = ((l - Point3d.Origin) * (1.0 - u)) +
                             ((r - Point3d.Origin) * u) +
                             ((b - Point3d.Origin) * (1.0 - v)) +
                             ((t - Point3d.Origin) * v) -
                             (bilinear - Point3d.Origin);

            return Point3d.Origin + blend;
        }

        private static Point3d EvaluateLayerPoint(LayerData layer, double u, double v)
        {
            int sideDenseCount = Math.Max(1, layer.DensePerimeterPoints.Count / 4);
            List<Point3d> bottomDense = ExtractSidePoints(layer.DensePerimeterPoints, 0, sideDenseCount);
            List<Point3d> rightDense = ExtractSidePoints(layer.DensePerimeterPoints, sideDenseCount, sideDenseCount);
            List<Point3d> topDense = ExtractSidePoints(layer.DensePerimeterPoints, sideDenseCount * 2, sideDenseCount);
            List<Point3d> leftDense = ExtractSidePoints(layer.DensePerimeterPoints, sideDenseCount * 3, sideDenseCount);

            Point3d bottom = EvaluateDenseSideAt(bottomDense, u);
            Point3d right = EvaluateDenseSideAt(rightDense, v);
            Point3d top = EvaluateDenseSideAt(topDense, 1.0 - u);
            Point3d left = EvaluateDenseSideAt(leftDense, 1.0 - v);

            Point3d c00 = bottomDense[0];
            Point3d c10 = bottomDense[bottomDense.Count - 1];
            Point3d c01 = topDense[topDense.Count - 1];
            Point3d c11 = topDense[0];

            return EvaluateCoonsPatch(bottom, top, left, right, c00, c10, c01, c11, u, v);
        }

        private static int GridIndex(int uIndex, int vIndex, int width)
        {
            return (vIndex * width) + uIndex;
        }

        private static List<Point3d> FlattenGrid(Point3d[,] grid, int uCount, int vCount)
        {
            var points = new List<Point3d>(uCount * vCount);
            for (int vIndex = 0; vIndex < vCount; vIndex++)
            {
                for (int uIndex = 0; uIndex < uCount; uIndex++)
                {
                    points.Add(grid[uIndex, vIndex]);
                }
            }

            return points;
        }

        private static string BuildAnalysis(
            IReadOnlyList<LayerData> layers,
            int baseGridResolution,
            int finalUCount,
            int finalVCount,
            int boundaryDivisionCount,
            double adaptiveStrength,
            int adaptiveLevels,
            double adaptiveDeviationThreshold,
            int interLayerStrutCount,
            int cellCenterCount,
            IReadOnlyList<string> notes)
        {
            var lines = new List<string>
            {
                "Layers: " + layers.Count,
                "Base grid resolution: " + baseGridResolution + " x " + baseGridResolution,
                "Adaptive grid resolution: " + finalUCount + " x " + finalVCount,
                "Boundary divisions per layer: " + boundaryDivisionCount,
                "Adaptive strength: " + adaptiveStrength.ToString("0.###"),
                "Adaptive levels: " + adaptiveLevels,
                "Deviation threshold: " + adaptiveDeviationThreshold.ToString("0.###"),
                "Layer meshes: " + layers.Count,
                "Inter-layer struts: " + interLayerStrutCount,
                "Cell centers: " + cellCenterCount,
                "Base mapping: square parameter grid warped to each normalized nonplanar perimeter.",
                "Adaptive refinement: extra rows and columns are inserted only where boundary approximation deviation exceeds the threshold."
            };

            if (notes.Count > 0)
            {
                lines.Add("Notes: " + string.Join(" | ", notes.Distinct()));
            }

            return string.Join(Environment.NewLine, lines);
        }

        private static IReadOnlyList<double> MergeParameterSets(IReadOnlyList<double> first, IReadOnlyList<double> second)
        {
            var merged = first
                .Concat(second)
                .Select(value => Math.Max(0.0, Math.Min(1.0, value)))
                .Distinct(new ParameterComparer())
                .OrderBy(value => value)
                .ToList();

            if (merged.Count == 0)
            {
                return new[] { 0.0, 1.0 };
            }

            merged[0] = 0.0;
            merged[merged.Count - 1] = 1.0;
            return merged;
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
                int sampleCount = Math.Max(3, (int)Math.Round(curve.GetLength() / safeSegmentLength));

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
            return projected.Unitize() ? projected : Vector3d.Unset;
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
            double secondScore = ComputePlanarDeviation(previousRing[1], currentRing[1]);
            double lastScore = ComputePlanarDeviation(previousRing[lastIndex], currentRing[lastIndex]);
            return secondScore + lastScore;
        }

        private static double ComputePlanarDeviation(Point3d a, Point3d b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;
            return Math.Sqrt((dx * dx) + (dy * dy));
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

        private static string BuildUndirectedPointPairKey(Point3d a, Point3d b, double tolerance)
        {
            string pointA = QuantizePoint(a, tolerance);
            string pointB = QuantizePoint(b, tolerance);
            return string.CompareOrdinal(pointA, pointB) <= 0 ? pointA + "|" + pointB : pointB + "|" + pointA;
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
                IntersectionEvent best = intersections[0];
                double bestDistance = double.MaxValue;

                foreach (IntersectionEvent intersection in intersections)
                {
                    double distance = plane.DistanceTo(intersection.PointA);
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        best = intersection;
                    }
                }

                return best.ParameterA;
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

        private static PolylineCurve CreateOpenPolylineCurve(IReadOnlyList<Point3d> points)
        {
            return new PolylineCurve(new Polyline(points));
        }

        private static double NormalizeUnitParameter(double parameter)
        {
            double normalized = parameter % 1.0;
            return normalized < 0.0 ? normalized + 1.0 : normalized;
        }

        private sealed class LayerData
        {
            public LayerData(int stackIndex, Plane plane, List<Point3d> perimeterPoints)
            {
                StackIndex = stackIndex;
                Plane = plane;
                SourcePerimeterPoints = new List<Point3d>(perimeterPoints);
                PerimeterPoints = new List<Point3d>(perimeterPoints);
                DensePerimeterPoints = new List<Point3d>(perimeterPoints);
            }

            public int StackIndex { get; }
            public Plane Plane { get; }
            public List<Point3d> SourcePerimeterPoints { get; }
            public List<Point3d> PerimeterPoints { get; private set; }
            public List<Point3d> DensePerimeterPoints { get; private set; }
            public Mesh? Mesh { get; set; }
            public List<Point3d>? GridPoints { get; set; }
            public int UCount { get; set; }
            public int VCount { get; set; }
            public IReadOnlyList<double> UParameters { get; set; } = Array.Empty<double>();
            public IReadOnlyList<double> VParameters { get; set; } = Array.Empty<double>();

            public void NormalizePerimeter(int divisionCount, int denseDivisionCount, double seamParameter)
            {
                var sampler = new TopologicalRingSampler(SourcePerimeterPoints);
                PerimeterPoints = sampler.Sample(divisionCount, seamParameter);
                DensePerimeterPoints = sampler.Sample(Math.Max(denseDivisionCount, divisionCount), seamParameter);
            }

            public void ReversePreservingStart()
            {
                PerimeterPoints = CreateReversePreservingStartCopy(PerimeterPoints);
                DensePerimeterPoints = CreateReversePreservingStartCopy(DensePerimeterPoints);
            }

            public Vector3d GetStartTraversalDirection()
            {
                return new TopologicalRingSampler(PerimeterPoints).TangentAt(0.0);
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
                var points = new List<Point3d>(count);
                double normalizedSeam = NormalizeUnitParameter(seamParameter);
                for (int index = 0; index < count; index++)
                {
                    points.Add(Evaluate(normalizedSeam + ((double)index / count)));
                }

                return points;
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

        private sealed class ParameterComparer : IEqualityComparer<double>
        {
            public bool Equals(double x, double y)
            {
                return Math.Abs(x - y) <= 1e-6;
            }

            public int GetHashCode(double obj)
            {
                return Math.Round(obj * 1000000.0).GetHashCode();
            }
        }
    }
}
