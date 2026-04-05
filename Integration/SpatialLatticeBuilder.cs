using System;
using System.Collections.Generic;
using System.Linq;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Controls how adjacent layers are braced while building the spatial lattice graph.
    /// </summary>
    /// <remarks>
    /// The diagonal mode exists because some spatial-extrusion strategies prefer fewer crossings,
    /// while others benefit from stiffer cross-bracing. The value is consumed only by
    /// <see cref="SpatialLatticeBuilder"/> and does not modify any global plugin state.
    /// </remarks>
    internal enum SpatialLatticeDiagonalMode
    {
        None = 0,
        Alternating = 1,
        Cross = 2,
        SameDirection = 3,
    }

    /// <summary>
    /// Groups the user-facing controls for the first-pass spatial lattice generator.
    /// </summary>
    /// <remarks>
    /// This options object exists to keep the Grasshopper component thin and to make the lattice builder
    /// easier to reuse from future components. Callers should provide values in the active Rhino model units.
    /// Side-effects: none.
    /// </remarks>
    internal sealed class SpatialLatticeOptions
    {
        /// <summary>
        /// Gets or sets the fallback spacing used when an input layer curve is not already a usable closed polyline.
        /// </summary>
        public double FallbackSegmentLength { get; set; } = 10.0;

        /// <summary>
        /// Gets or sets the shared perimeter division count used to normalize every layer.
        /// </summary>
        /// <remarks>
        /// A value less than three tells the builder to derive the count from the densest input layer instead.
        /// </remarks>
        public int NormalizedDivisionCount { get; set; }

        /// <summary>
        /// Gets or sets how many inward lattice rings are created from each perimeter layer.
        /// </summary>
        public int InteriorRingCount { get; set; } = 2;

        /// <summary>
        /// Gets or sets a multiplier for the inward ring spacing derived from the average perimeter segment length.
        /// </summary>
        public double InteriorStepScale { get; set; } = 1.0;

        /// <summary>
        /// Gets or sets the inter-layer bracing mode used between adjacent slices.
        /// </summary>
        public SpatialLatticeDiagonalMode DiagonalMode { get; set; } = SpatialLatticeDiagonalMode.Cross;

        /// <summary>
        /// Gets or sets an optional guide curve used to anchor the startpoint of every normalized layer.
        /// </summary>
        /// <remarks>
        /// When supplied, the builder attempts to intersect the guide with each layer plane and uses the closest perimeter location
        /// on that layer as the seam. If the guide does not intersect a layer plane, the builder falls back to the closest point.
        /// </remarks>
        public Curve? SeamGuideCurve { get; set; }

        /// <summary>
        /// Gets or sets an optional guide curve used to force the travel direction after the startpoint is fixed.
        /// </summary>
        /// <remarks>
        /// When supplied, this guide is evaluated on each layer plane to determine whether the normalized ring
        /// should advance clockwise or counterclockwise away from the seam.
        /// </remarks>
        public Curve? DirectionGuideCurve { get; set; }
    }

    /// <summary>
    /// Bundles the unit-cell geometry and diagnostics generated from a stack of sliced layer curves.
    /// </summary>
    internal sealed class SpatialLatticeResult
    {
        /// <summary>
        /// Initializes a new lattice result container.
        /// </summary>
        public SpatialLatticeResult(
            IReadOnlyList<Curve> perimeterCurves,
            IReadOnlyList<Curve> interiorCurves,
            IReadOnlyList<Line> members,
            IReadOnlyList<Curve> cellFaces,
            IReadOnlyList<Point3d> nodes,
            IReadOnlyList<int> layerDivisions,
            string analysis)
        {
            PerimeterCurves = perimeterCurves;
            InteriorCurves = interiorCurves;
            Members = members;
            CellFaces = cellFaces;
            Nodes = nodes;
            LayerDivisions = layerDivisions;
            Analysis = analysis;
        }

        public IReadOnlyList<Curve> PerimeterCurves { get; }
        public IReadOnlyList<Curve> InteriorCurves { get; }
        public IReadOnlyList<Line> Members { get; }
        public IReadOnlyList<Curve> CellFaces { get; }
        public IReadOnlyList<Point3d> Nodes { get; }
        public IReadOnlyList<int> LayerDivisions { get; }
        public string Analysis { get; }
    }

    /// <summary>
    /// Builds a first-pass stacked spatial lattice from ordered nonplanar layer curves.
    /// </summary>
    /// <remarks>
    /// The builder exists to convert sliced perimeter layers into a graph that can later support spatial-truss
    /// optimization and Eulerian-path exploration. The current implementation focuses on unit-cell construction,
    /// not path planning. Unexpected behavior to note: input order defines stack order; the builder does not sort layers.
    /// Side-effects: allocates Rhino geometry for the current solve only and does not add objects to the Rhino document.
    /// </remarks>
    internal static class SpatialLatticeBuilder
    {
        /// <summary>
        /// Converts a stack of ordered layer curves into perimeter rings, inward rings, and stacked truss members.
        /// </summary>
        /// <param name="layerCurves">Closed layer curves ordered from one slice to the next. The collection cannot be <see langword="null"/>.</param>
        /// <param name="layerPlanes">Optional representative planes matching the layer order. The collection may be empty.</param>
        /// <param name="options">Controls for fallback sampling, inward-ring generation, and inter-layer bracing. Cannot be <see langword="null"/>.</param>
        /// <param name="tolerance">Geometric tolerance used for closing curves, deduplicating members, and intersection tests. Must be greater than zero.</param>
        /// <returns>
        /// A <see cref="SpatialLatticeResult"/> describing the current lattice graph. If no valid closed layers are found,
        /// the result is returned with empty geometry collections and diagnostic text explaining what happened.
        /// </returns>
        public static SpatialLatticeResult Build(
            IReadOnlyList<Curve> layerCurves,
            IReadOnlyList<Plane> layerPlanes,
            SpatialLatticeOptions options,
            double tolerance)
        {
            var perimeterCurves = new List<Curve>();
            var interiorCurves = new List<Curve>();
            var members = new List<Line>();
            var cellFaces = new List<Curve>();
            var layerDivisions = new List<int>();
            var notes = new List<string>();

            if (layerCurves == null || layerCurves.Count == 0)
            {
                return new SpatialLatticeResult(
                    perimeterCurves,
                    interiorCurves,
                    members,
                    cellFaces,
                    Array.Empty<Point3d>(),
                    layerDivisions,
                    "No layer curves were supplied. Input order defines stack order for this component.");
            }

            double safeTolerance = Math.Max(tolerance, 1e-6);
            var baseLayers = new List<LayerData>();

            for (int index = 0; index < layerCurves.Count; index++)
            {
                Curve curve = layerCurves[index];
                Plane? plane = index < layerPlanes.Count ? layerPlanes[index] : (Plane?)null;
                LayerData? layer = TryBuildLayer(baseLayers.Count, curve, plane, options, safeTolerance, notes);
                if (layer == null)
                {
                    continue;
                }

                baseLayers.Add(layer);
                layerDivisions.Add(layer.BaseDivisionCount);
            }

            if (baseLayers.Count == 0)
            {
                return new SpatialLatticeResult(
                    perimeterCurves,
                    interiorCurves,
                    members,
                    cellFaces,
                    Array.Empty<Point3d>(),
                    layerDivisions,
                    "No valid closed layers were available for lattice generation.");
            }

            NormalizeLayerStack(baseLayers, options, safeTolerance, notes);

            int normalizedDivisionCount = Math.Max(
                3,
                options.NormalizedDivisionCount > 2
                    ? options.NormalizedDivisionCount
                    : baseLayers.Max(layer => layer.BaseDivisionCount));

            foreach (LayerData layer in baseLayers)
            {
                double seamParameter = ResolveLayerSeamParameter(layer, options.SeamGuideCurve, safeTolerance);
                layer.NormalizeDivisions(normalizedDivisionCount, seamParameter);
            }

            EnforceNormalizedDirection(baseLayers, options.SeamGuideCurve, options.DirectionGuideCurve, safeTolerance, notes);

            List<double> globalParameters = Enumerable.Range(0, normalizedDivisionCount)
                .Select(index => (double)index / normalizedDivisionCount)
                .ToList();

            foreach (LayerData layer in baseLayers)
            {
                layer.FinalParameters = new List<double>(globalParameters);
                layer.ExpandedRings = new List<List<Point3d>>(layer.BaseSamplers.Count);
                layer.ExpandedRingCurves = new List<Curve>(layer.BaseSamplers.Count);

                foreach (TopologicalRingSampler sampler in layer.BaseSamplers)
                {
                    List<Point3d> expandedRing = SampleRingByParameters(sampler, layer.FinalParameters);
                    layer.ExpandedRings.Add(expandedRing);
                    layer.ExpandedRingCurves.Add(CreateClosedPolylineCurve(expandedRing));
                }

                perimeterCurves.Add(layer.ExpandedRingCurves[0]);
                interiorCurves.AddRange(layer.ExpandedRingCurves.Skip(1));
            }

            var memberKeys = new HashSet<string>(StringComparer.Ordinal);
            int ringEdgeCount = 0;
            int spokeCount = 0;
            int verticalCount = 0;
            int diagonalCount = 0;

            foreach (LayerData layer in baseLayers)
            {
                AddWithinLayerMembers(layer, members, memberKeys, safeTolerance, ref ringEdgeCount, ref spokeCount);
            }

            for (int index = 0; index < baseLayers.Count - 1; index++)
            {
                AddBetweenLayerMembers(
                    baseLayers[index],
                    baseLayers[index + 1],
                    globalParameters,
                    options.DiagonalMode,
                    members,
                    cellFaces,
                    memberKeys,
                    safeTolerance,
                    ref verticalCount,
                    ref diagonalCount);
            }

            IReadOnlyList<Point3d> nodes = CollectNodes(baseLayers, members, safeTolerance);
            string analysis = BuildAnalysis(
                baseLayers,
                nodes.Count,
                members.Count,
                cellFaces.Count,
                ringEdgeCount,
                spokeCount,
                verticalCount,
                diagonalCount,
                notes);

            return new SpatialLatticeResult(perimeterCurves, interiorCurves, members, cellFaces, nodes, layerDivisions, analysis);
        }

        private static void NormalizeLayerStack(
            IReadOnlyList<LayerData> layers,
            SpatialLatticeOptions options,
            double tolerance,
            ICollection<string> notes)
        {
            if (layers.Count == 0)
            {
                return;
            }

            Vector3d referenceTraversal = ResolveReferenceTraversalDirection(
                layers[0],
                options.SeamGuideCurve,
                options.DirectionGuideCurve,
                tolerance);

            foreach (LayerData layer in layers)
            {
                if (layer.BaseRings.Count == 0)
                {
                    continue;
                }

                double seamParameter = ResolveLayerSeamParameter(layer, options.SeamGuideCurve, tolerance);
                Vector3d desiredTraversal = ResolveLayerTraversalDirection(
                    layer,
                    options.SeamGuideCurve,
                    options.DirectionGuideCurve,
                    seamParameter,
                    referenceTraversal,
                    tolerance);
                Vector3d actualTraversal = GetLayerTraversalDirection(layer, seamParameter);

                bool reversed = false;
                if (!desiredTraversal.IsTiny() && !actualTraversal.IsTiny() && (desiredTraversal * actualTraversal) < 0.0)
                {
                    layer.ApplyAlignment(true, 0);
                    reversed = true;
                }

                seamParameter = ResolveLayerSeamParameter(layer, options.SeamGuideCurve, tolerance);
                if (!reversed && seamParameter <= 1e-6)
                {
                    continue;
                }

                var noteParts = new List<string> { "Normalized layer " + (layer.StackIndex + 1) };
                if (reversed)
                {
                    noteParts.Add("reversed winding");
                }

                if (seamParameter > 1e-6)
                {
                    noteParts.Add("applied seam parameter " + seamParameter.ToString("0.###"));
                }

                notes.Add(string.Join(", ", noteParts));
            }
        }

        private static void EnforceNormalizedDirection(
            IReadOnlyList<LayerData> layers,
            Curve? seamGuideCurve,
            Curve? directionGuideCurve,
            double tolerance,
            ICollection<string> notes)
        {
            if (layers.Count == 0)
            {
                return;
            }

            Vector3d firstLayerDesiredDirection = ResolvePostNormalizationReferenceDirection(
                layers[0],
                seamGuideCurve,
                directionGuideCurve,
                tolerance);
            Vector3d firstLayerActualDirection = layers[0].GetStartTraversalDirection();
            if (!firstLayerDesiredDirection.IsTiny() &&
                !firstLayerActualDirection.IsTiny() &&
                (firstLayerDesiredDirection * firstLayerActualDirection) < 0.0)
            {
                layers[0].ReversePreservingStart();
                notes.Add("Flipped normalized direction on layer 1 to match the guide direction.");
            }

            for (int index = 1; index < layers.Count; index++)
            {
                LayerData previousLayer = layers[index - 1];
                LayerData currentLayer = layers[index];
                if (previousLayer.BaseRings.Count == 0 || currentLayer.BaseRings.Count == 0)
                {
                    continue;
                }

                double asIsScore = ComputeSecondPointVerticalityScore(previousLayer, currentLayer, false);
                double reversedScore = ComputeSecondPointVerticalityScore(previousLayer, currentLayer, true);
                if (reversedScore + Math.Max(tolerance, 1e-6) < asIsScore)
                {
                    currentLayer.ReversePreservingStart();
                    notes.Add("Flipped normalized direction on layer " + (currentLayer.StackIndex + 1) + " because the second-point test produced a more vertical correspondence.");
                }
            }
        }

        private static LayerData? TryBuildLayer(
            int stackIndex,
            Curve curve,
            Plane? suppliedPlane,
            SpatialLatticeOptions options,
            double tolerance,
            ICollection<string> notes)
        {
            if (curve == null || !curve.IsValid)
            {
                notes.Add("Skipped an invalid layer curve.");
                return null;
            }

            List<Point3d> perimeterPoints = ExtractClosedLayerPoints(curve, options.FallbackSegmentLength, tolerance);
            if (perimeterPoints.Count < 3)
            {
                notes.Add("Skipped a layer because it was not a usable closed loop.");
                return null;
            }

            Plane plane = ResolveLayerPlane(perimeterPoints, suppliedPlane);
            List<Point3d> projectedPerimeter = ProjectPointsToPlane(perimeterPoints, plane);
            Curve projectedPerimeterCurve = CreateClosedPolylineCurve(projectedPerimeter);

            Point3d centerOnPlane = ComputePlanarCentroid(projectedPerimeter, plane);
            Point3d centerInSpace = AveragePoint(perimeterPoints);

            double averageSegmentLength = ComputeAverageClosedSegmentLength(projectedPerimeter);
            double stepDistance = Math.Max(tolerance * 4.0, averageSegmentLength * Math.Max(0.1, options.InteriorStepScale));

            var baseRings = new List<List<Point3d>> { perimeterPoints };
            int requestedRings = Math.Max(0, options.InteriorRingCount);

            for (int ringIndex = 1; ringIndex <= requestedRings; ringIndex++)
            {
                double offsetDistance = stepDistance * ringIndex;
                Curve? inwardOffset = TryCreateInwardOffset(projectedPerimeterCurve, plane, offsetDistance, tolerance);
                if (inwardOffset == null)
                {
                    break;
                }

                List<Point3d> ring = BuildInteriorRing(
                    perimeterPoints,
                    projectedPerimeter,
                    centerInSpace,
                    centerOnPlane,
                    inwardOffset,
                    offsetDistance,
                    tolerance);

                if (ring.Count < 3 || ComputeAverageClosedSegmentLength(ring) <= tolerance)
                {
                    break;
                }

                baseRings.Add(ring);
            }

            var samplers = baseRings
                .Select(ring => new TopologicalRingSampler(ring))
                .ToList();

            return new LayerData(stackIndex, plane, baseRings, samplers);
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
                double curveLength = curve.GetLength();
                double safeSegmentLength = Math.Max(fallbackSegmentLength, tolerance * 4.0);
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

            points = RemoveSequentialDuplicates(points, tolerance);
            if (points.Count < 3)
            {
                return new List<Point3d>();
            }

            bool closedEnough = curve.IsClosed || points[0].DistanceTo(points[points.Count - 1]) <= tolerance * 4.0;
            return closedEnough ? points : new List<Point3d>();
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

            Point3d center = AveragePoint(points);
            return new Plane(center, Vector3d.ZAxis);
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
            double centroidUAccumulator = 0.0;
            double centroidVAccumulator = 0.0;

            for (int index = 0; index < coordinates.Count; index++)
            {
                Point2d current = coordinates[index];
                Point2d next = coordinates[(index + 1) % coordinates.Count];
                double cross = (current.X * next.Y) - (next.X * current.Y);
                signedAreaTwice += cross;
                centroidUAccumulator += (current.X + next.X) * cross;
                centroidVAccumulator += (current.Y + next.Y) * cross;
            }

            if (Math.Abs(signedAreaTwice) <= 1e-9)
            {
                double averageU = coordinates.Average(point => point.X);
                double averageV = coordinates.Average(point => point.Y);
                return plane.PointAt(averageU, averageV);
            }

            double scale = 1.0 / (3.0 * signedAreaTwice);
            return plane.PointAt(centroidUAccumulator * scale, centroidVAccumulator * scale);
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

            double inverseCount = 1.0 / points.Count;
            return new Point3d(x * inverseCount, y * inverseCount, z * inverseCount);
        }

        private static double ComputeAverageClosedSegmentLength(IReadOnlyList<Point3d> ring)
        {
            if (ring.Count < 2)
            {
                return 0.0;
            }

            double total = 0.0;
            for (int index = 0; index < ring.Count; index++)
            {
                total += ring[index].DistanceTo(ring[(index + 1) % ring.Count]);
            }

            return total / ring.Count;
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

            IEnumerable<Curve> candidates = (positiveOffsets ?? Array.Empty<Curve>())
                .Concat(negativeOffsets ?? Array.Empty<Curve>());

            foreach (Curve candidate in candidates)
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

        private static List<Point3d> BuildInteriorRing(
            IReadOnlyList<Point3d> perimeterPoints,
            IReadOnlyList<Point3d> projectedPerimeter,
            Point3d centerInSpace,
            Point3d centerOnPlane,
            Curve inwardOffset,
            double offsetDistance,
            double tolerance)
        {
            var ring = new List<Point3d>(perimeterPoints.Count);

            for (int index = 0; index < perimeterPoints.Count; index++)
            {
                Point3d projectedPoint = projectedPerimeter[index];
                Point3d perimeterPoint = perimeterPoints[index];

                double projectedRadius = centerOnPlane.DistanceTo(projectedPoint);
                if (projectedRadius <= tolerance)
                {
                    ring.Add(centerInSpace);
                    continue;
                }

                Vector3d direction = projectedPoint - centerOnPlane;
                Point3d farPoint = centerOnPlane + (direction * 2.5);

                CurveIntersections intersections = Intersection.CurveLine(inwardOffset, new Line(centerOnPlane, farPoint), tolerance, tolerance);
                double bestDistance = 0.0;

                if (intersections != null)
                {
                    foreach (IntersectionEvent intersection in intersections)
                    {
                        Point3d candidate = intersection.PointA;
                        Vector3d candidateDirection = candidate - centerOnPlane;
                        if (direction * candidateDirection < 0.0)
                        {
                            continue;
                        }

                        double distance = centerOnPlane.DistanceTo(candidate);
                        if (distance > bestDistance && distance < projectedRadius + (tolerance * 8.0))
                        {
                            bestDistance = distance;
                        }
                    }
                }

                double alpha = bestDistance > 0.0
                    ? bestDistance / projectedRadius
                    : Math.Max(0.0, (projectedRadius - offsetDistance) / projectedRadius);

                alpha = Math.Max(0.0, Math.Min(1.0, alpha));
                ring.Add(centerInSpace + ((perimeterPoint - centerInSpace) * alpha));
            }

            return RemoveSequentialDuplicates(ring, tolerance);
        }

        private static double ComputeSignedAreaOnPlane(IReadOnlyList<Point3d> ring, Plane plane)
        {
            if (ring.Count < 3)
            {
                return 0.0;
            }

            double signedAreaTwice = 0.0;
            for (int index = 0; index < ring.Count; index++)
            {
                double currentU;
                double currentV;
                double nextU;
                double nextV;
                plane.ClosestParameter(ring[index], out currentU, out currentV);
                plane.ClosestParameter(ring[(index + 1) % ring.Count], out nextU, out nextV);
                signedAreaTwice += (currentU * nextV) - (nextU * currentV);
            }

            return signedAreaTwice * 0.5;
        }

        private static Vector3d ResolveReferenceTraversalDirection(
            LayerData layer,
            Curve? seamGuideCurve,
            Curve? directionGuideCurve,
            double tolerance)
        {
            double seamParameter = ResolveLayerSeamParameter(layer, seamGuideCurve, tolerance);
            Vector3d desired = ResolveLayerTraversalDirection(
                layer,
                seamGuideCurve,
                directionGuideCurve,
                seamParameter,
                Vector3d.Unset,
                tolerance);
            if (!desired.IsTiny())
            {
                return desired;
            }

            Vector3d fallback = GetLayerTraversalDirection(layer, seamParameter);
            if (!fallback.IsTiny())
            {
                return fallback;
            }

            return ProjectDirectionToPlane(Vector3d.XAxis, layer.Plane);
        }

        private static Vector3d ResolvePostNormalizationReferenceDirection(
            LayerData layer,
            Curve? seamGuideCurve,
            Curve? directionGuideCurve,
            double tolerance)
        {
            Vector3d desired = ResolvePostNormalizationDirection(
                layer,
                seamGuideCurve,
                directionGuideCurve,
                Vector3d.Unset,
                tolerance);
            if (!desired.IsTiny())
            {
                return desired;
            }

            Vector3d actual = layer.GetStartTraversalDirection();
            if (!actual.IsTiny())
            {
                return actual;
            }

            return ProjectDirectionToPlane(Vector3d.XAxis, layer.Plane);
        }

        private static Vector3d ResolveLayerTraversalDirection(
            LayerData layer,
            Curve? seamGuideCurve,
            Curve? directionGuideCurve,
            double seamParameter,
            Vector3d fallbackDirection,
            double tolerance)
        {
            Vector3d guideDirection = ResolveGuideDirection(layer.Plane, directionGuideCurve, tolerance);
            if (guideDirection.IsTiny())
            {
                guideDirection = ResolveGuideDirection(layer.Plane, seamGuideCurve, tolerance);
            }

            if (!guideDirection.IsTiny())
            {
                return guideDirection;
            }

            if (!fallbackDirection.IsTiny())
            {
                Vector3d projectedFallback = ProjectDirectionToPlane(fallbackDirection, layer.Plane);
                if (!projectedFallback.IsTiny())
                {
                    return projectedFallback;
                }
            }

            Vector3d ringTraversal = GetLayerTraversalDirection(layer, seamParameter);
            if (!ringTraversal.IsTiny())
            {
                return ringTraversal;
            }

            return ProjectDirectionToPlane(Vector3d.XAxis, layer.Plane);
        }

        private static Vector3d ResolvePostNormalizationDirection(
            LayerData layer,
            Curve? seamGuideCurve,
            Curve? directionGuideCurve,
            Vector3d fallbackDirection,
            double tolerance)
        {
            Vector3d guideDirection = ResolveGuideDirection(layer.Plane, directionGuideCurve, tolerance);
            if (guideDirection.IsTiny())
            {
                guideDirection = ResolveGuideDirection(layer.Plane, seamGuideCurve, tolerance);
            }

            if (!guideDirection.IsTiny())
            {
                return guideDirection;
            }

            if (!fallbackDirection.IsTiny())
            {
                Vector3d projectedFallback = ProjectDirectionToPlane(fallbackDirection, layer.Plane);
                if (!projectedFallback.IsTiny())
                {
                    return projectedFallback;
                }
            }

            return layer.GetStartTraversalDirection();
        }

        private static Vector3d ResolveGuideDirection(Plane plane, Curve? guideCurve, double tolerance)
        {
            if (guideCurve == null || !guideCurve.IsValid)
            {
                return Vector3d.Unset;
            }

            double guideParameter = FindGuideIntersectionParameter(guideCurve, plane, tolerance);
            Vector3d guideTangent = guideCurve.TangentAt(guideParameter);
            if (!guideTangent.Unitize())
            {
                return Vector3d.Unset;
            }

            Vector3d projectedGuide = ProjectDirectionToPlane(guideTangent, plane);
            return projectedGuide.IsTiny() ? Vector3d.Unset : projectedGuide;
        }

        private static double ComputeSecondPointVerticalityScore(LayerData previousLayer, LayerData currentLayer, bool reverseCurrent)
        {
            IReadOnlyList<Point3d> previousRing = previousLayer.BaseRings[0];
            IReadOnlyList<Point3d> currentRing = reverseCurrent
                ? CreateReversePreservingStartCopy(currentLayer.BaseRings[0])
                : currentLayer.BaseRings[0];

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

        private static double ResolveLayerSeamParameter(LayerData layer, Curve? seamGuideCurve, double tolerance)
        {
            if (layer.BaseRings.Count == 0)
            {
                return 0.0;
            }

            Curve perimeterCurve = CreateClosedPolylineCurve(layer.BaseRings[0]);
            if (!perimeterCurve.IsValid || perimeterCurve.GetLength() <= tolerance)
            {
                return 0.0;
            }

            if (seamGuideCurve != null && seamGuideCurve.IsValid)
            {
                double guideParameter = FindGuideIntersectionParameter(seamGuideCurve, layer.Plane, tolerance);
                Point3d guidePoint = seamGuideCurve.PointAt(guideParameter);
                return FindCurveNormalizedLengthAtClosestPoint(perimeterCurve, guidePoint);
            }

            int anchorIndex = FindDirectionalAnchorIndex(layer.BaseRings[0], layer.Plane);
            if (anchorIndex <= 0 || anchorIndex >= layer.BaseSamplers[0].VertexParameters.Count)
            {
                return 0.0;
            }

            return layer.BaseSamplers[0].VertexParameters[anchorIndex];
        }

        private static Vector3d GetLayerTraversalDirection(LayerData layer, double seamParameter)
        {
            if (layer.BaseSamplers.Count == 0)
            {
                return Vector3d.Unset;
            }

            Vector3d tangent = layer.BaseSamplers[0].TangentAt(seamParameter);
            if (!tangent.Unitize())
            {
                return Vector3d.Unset;
            }

            return tangent;
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

            Point3d sampleOrigin = plane.Origin;
            if (guideCurve.ClosestPoint(sampleOrigin, out double closestGuideParameter))
            {
                return closestGuideParameter;
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
            double partialLength = curve.GetLength(partialDomain);
            double normalized = partialLength / totalLength;
            return NormalizeUnitParameter(normalized);
        }

        private static int FindDirectionalAnchorIndex(IReadOnlyList<Point3d> ring, Plane plane)
        {
            if (ring.Count == 0)
            {
                return 0;
            }

            Vector3d localDirection = ProjectDirectionToPlane(Vector3d.XAxis, plane);
            if (localDirection.IsTiny())
            {
                localDirection = ProjectDirectionToPlane(Vector3d.YAxis, plane);
            }

            if (localDirection.IsTiny())
            {
                localDirection = plane.XAxis;
            }

            Point3d centroid = AveragePoint(ring);
            int bestIndex = 0;
            double bestScore = double.MinValue;

            for (int index = 0; index < ring.Count; index++)
            {
                Vector3d fromCenter = ring[index] - centroid;
                double score = fromCenter * localDirection;
                if (score > bestScore)
                {
                    bestScore = score;
                    bestIndex = index;
                }
            }

            return bestIndex;
        }

        private static Vector3d ProjectDirectionToPlane(Vector3d direction, Plane plane)
        {
            Vector3d projected = direction - ((direction * plane.Normal) * plane.Normal);
            projected.Unitize();
            return projected;
        }

        private static double NormalizeUnitParameter(double value)
        {
            double normalized = value % 1.0;
            if (normalized < 0.0)
            {
                normalized += 1.0;
            }

            return Math.Abs(normalized - 1.0) <= 1e-9 ? 0.0 : normalized;
        }

        private static List<Point3d> SampleRingByParameters(TopologicalRingSampler sampler, IReadOnlyList<double> parameters)
        {
            var samples = new List<Point3d>(parameters.Count);
            foreach (double parameter in parameters)
            {
                samples.Add(sampler.Evaluate(parameter));
            }

            return samples;
        }

        private static void AddWithinLayerMembers(
            LayerData layer,
            ICollection<Line> members,
            ISet<string> memberKeys,
            double tolerance,
            ref int ringEdgeCount,
            ref int spokeCount)
        {
            foreach (List<Point3d> ring in layer.ExpandedRings)
            {
                for (int index = 0; index < ring.Count; index++)
                {
                    if (TryAddMember(ring[index], ring[(index + 1) % ring.Count], members, memberKeys, tolerance))
                    {
                        ringEdgeCount++;
                    }
                }
            }

            for (int ringIndex = 0; ringIndex < layer.ExpandedRings.Count - 1; ringIndex++)
            {
                List<Point3d> outerRing = layer.ExpandedRings[ringIndex];
                List<Point3d> innerRing = layer.ExpandedRings[ringIndex + 1];

                int count = Math.Min(outerRing.Count, innerRing.Count);
                for (int index = 0; index < count; index++)
                {
                    if (TryAddMember(outerRing[index], innerRing[index], members, memberKeys, tolerance))
                    {
                        spokeCount++;
                    }
                }
            }
        }

        private static void AddBetweenLayerMembers(
            LayerData lowerLayer,
            LayerData upperLayer,
            IReadOnlyList<double> sharedParameters,
            SpatialLatticeDiagonalMode diagonalMode,
            ICollection<Line> members,
            ICollection<Curve> cellFaces,
            ISet<string> memberKeys,
            double tolerance,
            ref int verticalCount,
            ref int diagonalCount)
        {
            int ringCount = Math.Min(lowerLayer.BaseSamplers.Count, upperLayer.BaseSamplers.Count);
            if (ringCount == 0 || sharedParameters.Count < 2)
            {
                return;
            }

            for (int ringIndex = 0; ringIndex < ringCount; ringIndex++)
            {
                List<Point3d> lowerSamples = SampleRingByParameters(lowerLayer.BaseSamplers[ringIndex], sharedParameters);
                List<Point3d> upperSamples = SampleRingByParameters(upperLayer.BaseSamplers[ringIndex], sharedParameters);

                for (int index = 0; index < sharedParameters.Count; index++)
                {
                    int next = (index + 1) % sharedParameters.Count;

                    if (TryAddMember(lowerSamples[index], upperSamples[index], members, memberKeys, tolerance))
                    {
                        verticalCount++;
                    }

                    Curve? face = CreateQuadFace(lowerSamples[index], lowerSamples[next], upperSamples[next], upperSamples[index], tolerance);
                    if (face != null)
                    {
                        cellFaces.Add(face);
                    }

                    if (diagonalMode == SpatialLatticeDiagonalMode.None)
                    {
                        continue;
                    }

                    if (diagonalMode == SpatialLatticeDiagonalMode.Cross)
                    {
                        if (TryAddMember(lowerSamples[index], upperSamples[next], members, memberKeys, tolerance))
                        {
                            diagonalCount++;
                        }

                        if (TryAddMember(lowerSamples[next], upperSamples[index], members, memberKeys, tolerance))
                        {
                            diagonalCount++;
                        }

                        continue;
                    }

                    bool useForwardDiagonal = diagonalMode == SpatialLatticeDiagonalMode.SameDirection
                        || ((lowerLayer.StackIndex + ringIndex + index) % 2) == 0;
                    Point3d diagonalStart = useForwardDiagonal ? lowerSamples[index] : lowerSamples[next];
                    Point3d diagonalEnd = useForwardDiagonal ? upperSamples[next] : upperSamples[index];
                    if (TryAddMember(diagonalStart, diagonalEnd, members, memberKeys, tolerance))
                    {
                        diagonalCount++;
                    }
                }
            }
        }

        private static Curve? CreateQuadFace(Point3d a, Point3d b, Point3d c, Point3d d, double tolerance)
        {
            if (a.DistanceTo(b) <= tolerance || b.DistanceTo(c) <= tolerance || c.DistanceTo(d) <= tolerance || d.DistanceTo(a) <= tolerance)
            {
                return null;
            }

            var polyline = new Polyline(new[] { a, b, c, d, a });
            if (!polyline.IsValid || polyline.Count < 5)
            {
                return null;
            }

            return new PolylineCurve(polyline);
        }

        private static bool TryAddMember(
            Point3d start,
            Point3d end,
            ICollection<Line> members,
            ISet<string> memberKeys,
            double tolerance)
        {
            if (start.DistanceTo(end) <= tolerance)
            {
                return false;
            }

            string key = BuildUndirectedEdgeKey(start, end, tolerance);
            if (!memberKeys.Add(key))
            {
                return false;
            }

            members.Add(new Line(start, end));
            return true;
        }

        private static string BuildUndirectedEdgeKey(Point3d a, Point3d b, double tolerance)
        {
            string keyA = BuildPointKey(a, tolerance);
            string keyB = BuildPointKey(b, tolerance);
            return string.CompareOrdinal(keyA, keyB) <= 0 ? keyA + "|" + keyB : keyB + "|" + keyA;
        }

        private static string BuildPointKey(Point3d point, double tolerance)
        {
            double cell = Math.Max(tolerance, 1e-6);
            long x = (long)Math.Round(point.X / cell);
            long y = (long)Math.Round(point.Y / cell);
            long z = (long)Math.Round(point.Z / cell);
            return x + "," + y + "," + z;
        }

        private static IReadOnlyList<Point3d> CollectNodes(IReadOnlyList<LayerData> layers, IReadOnlyList<Line> members, double tolerance)
        {
            var nodes = new List<Point3d>();
            var keys = new HashSet<string>(StringComparer.Ordinal);

            foreach (LayerData layer in layers)
            {
                foreach (List<Point3d> ring in layer.ExpandedRings)
                {
                    foreach (Point3d point in ring)
                    {
                        if (keys.Add(BuildPointKey(point, tolerance)))
                        {
                            nodes.Add(point);
                        }
                    }
                }
            }

            foreach (Line member in members)
            {
                if (keys.Add(BuildPointKey(member.From, tolerance)))
                {
                    nodes.Add(member.From);
                }

                if (keys.Add(BuildPointKey(member.To, tolerance)))
                {
                    nodes.Add(member.To);
                }
            }

            return nodes;
        }

        private static string BuildAnalysis(
            IReadOnlyList<LayerData> layers,
            int nodeCount,
            int memberCount,
            int cellFaceCount,
            int ringEdgeCount,
            int spokeCount,
            int verticalCount,
            int diagonalCount,
            IReadOnlyCollection<string> notes)
        {
            int minimumBuiltRings = layers.Count == 0 ? 0 : layers.Min(layer => layer.BaseSamplers.Count);
            int maximumBuiltRings = layers.Count == 0 ? 0 : layers.Max(layer => layer.BaseSamplers.Count);
            string baseDivisions = string.Join(", ", layers.Select(layer => layer.BaseDivisionCount));
            string expandedDivisions = string.Join(", ", layers.Select(layer => layer.FinalParameters.Count));
            int normalizedDivisionCount = layers.Count == 0 ? 0 : layers[0].FinalParameters.Count;

            var lines = new List<string>
            {
                "Spatial lattice unit-cell scaffold",
                "Input order defines stack order.",
                "Layers processed: " + layers.Count,
                "Base perimeter divisions: " + baseDivisions,
                "Normalized perimeter divisions: " + normalizedDivisionCount,
                "Expanded graph nodes per layer: " + expandedDivisions,
                "Built rings per layer (including perimeter): min " + minimumBuiltRings + ", max " + maximumBuiltRings,
                "Nodes: " + nodeCount,
                "Members: " + memberCount,
                "Cell faces: " + cellFaceCount,
                "Ring edges: " + ringEdgeCount,
                "Radial spokes: " + spokeCount,
                "Vertical struts: " + verticalCount,
                "Diagonals: " + diagonalCount,
            };

            if (notes.Count > 0)
            {
                lines.Add("Notes: " + string.Join(" | ", notes.Distinct()));
            }

            return string.Join(Environment.NewLine, lines);
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

        private static List<Point3d> RotateRing(IReadOnlyList<Point3d> points, int shift)
        {
            if (points.Count == 0)
            {
                return new List<Point3d>();
            }

            int normalizedShift = ((shift % points.Count) + points.Count) % points.Count;
            if (normalizedShift == 0)
            {
                return new List<Point3d>(points);
            }

            var rotated = new List<Point3d>(points.Count);
            for (int index = 0; index < points.Count; index++)
            {
                rotated.Add(points[(index + normalizedShift) % points.Count]);
            }

            return rotated;
        }

        private static List<Point3d> ReverseRing(IReadOnlyList<Point3d> points)
        {
            var reversed = new List<Point3d>(points);
            reversed.Reverse();
            return reversed;
        }

        private static List<Point3d> CreateReversePreservingStartCopy(IReadOnlyList<Point3d> points)
        {
            if (points.Count <= 2)
            {
                return new List<Point3d>(points);
            }

            Point3d start = points[0];
            var reversed = new List<Point3d>(points.Count) { start };
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
            public LayerData(int stackIndex, Plane plane, List<List<Point3d>> baseRings, List<TopologicalRingSampler> baseSamplers)
            {
                StackIndex = stackIndex;
                Plane = plane;
                BaseRings = baseRings;
                BaseSamplers = baseSamplers;
                FinalParameters = new List<double>(baseSamplers[0].VertexParameters);
                ExpandedRings = new List<List<Point3d>>();
                ExpandedRingCurves = new List<Curve>();
            }

            public int StackIndex { get; }
            public Plane Plane { get; }
            public List<List<Point3d>> BaseRings { get; }
            public List<TopologicalRingSampler> BaseSamplers { get; private set; }
            public int BaseDivisionCount => BaseRings[0].Count;
            public List<double> FinalParameters { get; set; }
            public List<List<Point3d>> ExpandedRings { get; set; }
            public List<Curve> ExpandedRingCurves { get; set; }

            public void ApplyAlignment(bool reverse, int shift)
            {
                if (!reverse && shift == 0)
                {
                    return;
                }

                for (int ringIndex = 0; ringIndex < BaseRings.Count; ringIndex++)
                {
                    List<Point3d> ring = BaseRings[ringIndex];
                    if (reverse)
                    {
                        ring = ReverseRing(ring);
                    }

                    if (shift != 0)
                    {
                        ring = RotateRing(ring, shift);
                    }

                    BaseRings[ringIndex] = ring;
                }

                BaseSamplers = BaseRings
                    .Select(ring => new TopologicalRingSampler(ring))
                    .ToList();
            }

            public void NormalizeDivisions(int divisionCount, double seamParameter)
            {
                if (divisionCount < 3)
                {
                    return;
                }

                double normalizedSeam = NormalizeUnitParameter(seamParameter);
                for (int ringIndex = 0; ringIndex < BaseRings.Count; ringIndex++)
                {
                    var sampler = new TopologicalRingSampler(BaseRings[ringIndex]);
                    var resampledRing = new List<Point3d>(divisionCount);

                    for (int sampleIndex = 0; sampleIndex < divisionCount; sampleIndex++)
                    {
                        double parameter = normalizedSeam + ((double)sampleIndex / divisionCount);
                        resampledRing.Add(sampler.Evaluate(parameter));
                    }

                    BaseRings[ringIndex] = resampledRing;
                }

                BaseSamplers = BaseRings
                    .Select(ring => new TopologicalRingSampler(ring))
                    .ToList();
            }

            public Vector3d GetStartTraversalDirection()
            {
                if (BaseSamplers.Count == 0)
                {
                    return Vector3d.Unset;
                }

                return BaseSamplers[0].TangentAt(0.0);
            }

            public void ReversePreservingStart()
            {
                for (int ringIndex = 0; ringIndex < BaseRings.Count; ringIndex++)
                {
                    List<Point3d> ring = BaseRings[ringIndex];
                    BaseRings[ringIndex] = CreateReversePreservingStartCopy(ring);
                }

                BaseSamplers = BaseRings
                    .Select(ring => new TopologicalRingSampler(ring))
                    .ToList();
            }
        }

        /// <summary>
        /// Samples a closed ring by topological station rather than by arc length.
        /// </summary>
        private sealed class TopologicalRingSampler
        {
            private readonly IReadOnlyList<Point3d> _points;
            private readonly IReadOnlyList<double> _vertexParameters;

            public TopologicalRingSampler(IReadOnlyList<Point3d> points)
            {
                _points = points;
                _vertexParameters = BuildVertexParameters(points);
                VertexParameters = _vertexParameters;
            }

            public IReadOnlyList<double> VertexParameters { get; }

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
                double denominator = Math.Max(1e-9, endParameter - startParameter);
                double local = (parameter - startParameter) / denominator;
                return _points[startIndex] + ((_points[endIndex] - _points[startIndex]) * local);
            }

            public Vector3d TangentAt(double normalizedParameter)
            {
                if (_points.Count < 2)
                {
                    return Vector3d.Unset;
                }

                int startIndex;
                int endIndex;
                GetSegmentIndices(normalizedParameter, out startIndex, out endIndex);

                Vector3d tangent = _points[endIndex] - _points[startIndex];
                if (!tangent.Unitize())
                {
                    for (int offset = 1; offset < _points.Count; offset++)
                    {
                        int fallbackStart = (startIndex + offset) % _points.Count;
                        int fallbackEnd = (fallbackStart + 1) % _points.Count;
                        tangent = _points[fallbackEnd] - _points[fallbackStart];
                        if (tangent.Unitize())
                        {
                            return tangent;
                        }
                    }

                    return Vector3d.Unset;
                }

                return tangent;
            }

            private void GetSegmentIndices(double normalizedParameter, out int startIndex, out int endIndex)
            {
                double parameter = NormalizeUnitParameter(normalizedParameter);
                startIndex = _points.Count - 1;
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

                endIndex = (startIndex + 1) % _points.Count;
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
                    return Enumerable.Range(0, points.Count)
                        .Select(index => (double)index / points.Count)
                        .ToArray();
                }

                var parameters = new double[points.Count];
                double cumulative = 0.0;
                parameters[0] = 0.0;

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
