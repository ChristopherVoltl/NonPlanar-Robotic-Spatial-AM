using System;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using Rhino.Geometry;
using SpatialAdditiveManufacturing.Core.Slicing;

namespace NonPlanar_Robotic_Spatial_AM
{
    internal sealed class RhinoLayerSliceResult
    {
        public RhinoLayerSliceResult(
            IReadOnlyList<Curve> curves,
            IReadOnlyList<Plane> planes,
            string analysis,
            Mesh anglePreviewMesh,
            IReadOnlyList<Line> fieldLines,
            string angleHistogram)
        {
            Curves = curves;
            Planes = planes;
            Analysis = analysis;
            AnglePreviewMesh = anglePreviewMesh;
            FieldLines = fieldLines;
            AngleHistogram = angleHistogram;
        }

        public IReadOnlyList<Curve> Curves { get; }
        public IReadOnlyList<Plane> Planes { get; }
        public string Analysis { get; }
        public Mesh AnglePreviewMesh { get; }
        public IReadOnlyList<Line> FieldLines { get; }
        public string AngleHistogram { get; }
    }

    /// <summary>
    /// Scalar-field slicer built on a Brep-derived analysis mesh.
    /// At Angle Influence = 0 the field reduces to global Z height, producing planar XY layers.
    /// Increasing Angle Influence blends the scalar field toward the selected Brep surface direction and curvature.
    /// </summary>
    internal static class RhinoBrepLayerSlicer
    {
        public static RhinoLayerSliceResult Slice(Brep brep, SliceGenerationOptions options, double tolerance)
        {
            var curves = new List<Curve>();
            var planes = new List<Plane>();

            Mesh analysisMesh = CreateAnalysisMesh(brep, options, tolerance);
            if (analysisMesh.Vertices.Count == 0 || analysisMesh.Faces.Count == 0)
            {
                return new RhinoLayerSliceResult(curves, planes, "Unable to create an analysis mesh from the Brep.", new Mesh(), Array.Empty<Line>(), "No histogram available.");
            }

            ScalarFieldData field = BuildScalarField(analysisMesh, options, tolerance);
            double minLevel = Math.Ceiling(field.Values.Min());
            double maxLevel = Math.Floor(field.Values.Max());

            if (maxLevel < minLevel)
            {
                Mesh emptyPreviewMesh = CreateAnglePreviewMesh(analysisMesh, field.AnglesDegrees);
                List<Line> fieldLines = CreateFieldLines(analysisMesh, field.FieldVectors, field.LayerHeight);
                string histogram = BuildAngleHistogram(field.AnglesDegrees);
                return new RhinoLayerSliceResult(curves, planes, "The scalar field did not span a full layer interval.", emptyPreviewMesh, fieldLines, histogram);
            }

            for (double level = minLevel; level <= maxLevel; level += 1.0)
            {
                Curve[] levelCurves = ExtractIsoCurves(analysisMesh, field.Values, level, tolerance);
                foreach (Curve curve in levelCurves)
                {
                    if (curve == null || !curve.IsValid || curve.GetLength() <= tolerance)
                    {
                        continue;
                    }

                    Curve outputCurve = SmoothCurveOnMesh(curve, analysisMesh, options, tolerance);
                    outputCurve = ConvertCurveToOutputPolyline(outputCurve, brep, analysisMesh, options, tolerance);
                    curves.Add(outputCurve);
                    planes.Add(BuildRepresentativePlane(outputCurve, analysisMesh, field.FieldVectors, tolerance));
                }
            }

            string analysis = RhinoSliceAnalysis.Format(curves, planes, field.LayerHeight, field.Values);
            Mesh anglePreviewMesh = CreateAnglePreviewMesh(analysisMesh, field.AnglesDegrees);
            List<Line> diagnosticFieldLines = CreateFieldLines(analysisMesh, field.FieldVectors, field.LayerHeight);
            string angleHistogram = BuildAngleHistogram(field.AnglesDegrees);
            return new RhinoLayerSliceResult(curves, planes, analysis, anglePreviewMesh, diagnosticFieldLines, angleHistogram);
        }

        private static Mesh CreateAnalysisMesh(Brep brep, SliceGenerationOptions options, double tolerance)
        {
            double layerHeight = Math.Max(options.TargetLayerHeight, tolerance * 2.0);
            var meshingParameters = new MeshingParameters
            {
                JaggedSeams = false,
                RefineGrid = true,
                SimplePlanes = false,
                MinimumEdgeLength = Math.Max(tolerance, layerHeight * 0.20),
                MaximumEdgeLength = Math.Max(layerHeight * 0.75, tolerance * 6.0),
                GridAmplification = 1.2,
                GridAspectRatio = 4.0
            };

            Mesh[]? brepMeshes = Mesh.CreateFromBrep(brep, meshingParameters);
            if (brepMeshes == null || brepMeshes.Length == 0)
            {
                return new Mesh();
            }

            var merged = new Mesh();
            foreach (Mesh mesh in brepMeshes)
            {
                if (mesh == null || !mesh.IsValid)
                {
                    continue;
                }

                merged.Append(mesh);
            }

            merged.Vertices.CombineIdentical(true, true);
            merged.Vertices.CullUnused();
            merged.Faces.CullDegenerateFaces();
            merged.Weld(Math.PI);
            merged.UnifyNormals();
            merged.FaceNormals.ComputeFaceNormals();
            merged.Normals.ComputeNormals();
            merged.Compact();
            return merged;
        }

        private static ScalarFieldData BuildScalarField(Mesh mesh, SliceGenerationOptions options, double tolerance)
        {
            int vertexCount = mesh.Vertices.Count;
            double layerHeight = Math.Max(options.TargetLayerHeight, tolerance * 2.0);
            List<int>[] adjacency = BuildAdjacency(mesh);
            Point3d[] vertices = Enumerable.Range(0, vertexCount)
                .Select(index => mesh.Vertices.Point3dAt(index))
                .ToArray();

            Vector3d[] normals = BuildVertexNormals(mesh, adjacency, options);
            double[] curvature = BuildSignedCurvature(vertices, normals, adjacency);
            double[] normalizedCurvature = NormalizeSignedValues(curvature);

            BoundingBox boundingBox = mesh.GetBoundingBox(true);
            double minZ = boundingBox.Min.Z;
            double[] baseField = vertices
                .Select(point => (point.Z - minZ) / layerHeight)
                .ToArray();

            Vector3d[] tangentTargets = BuildTangentTargets(vertices, normals, normalizedCurvature, adjacency, options);
            Vector3d[] directionTargets = BuildDirectionTargets(normals, tangentTargets, options);
            Vector3d[] fieldVectors = new Vector3d[vertexCount];
            double[] fieldAnglesDegrees = new double[vertexCount];
            double[] fieldValues = (double[])baseField.Clone();
            double globalInfluence = options.NormalizedAngleInfluence;

            for (int i = 0; i < vertexCount; i++)
            {
                double slopeWeight = 1.0 - Math.Abs(directionTargets[i] * Vector3d.ZAxis);
                double curvatureWeight = Math.Abs(normalizedCurvature[i]);
                double adaptiveBlend = Clamp01(options.AdaptiveBlendFactor);
                double localWeight = Lerp(slopeWeight, Math.Max(slopeWeight, curvatureWeight), adaptiveBlend);
                double localInfluence = globalInfluence * localWeight;

                fieldVectors[i] = BuildFieldVector(directionTargets[i], localInfluence);
                fieldAnglesDegrees[i] = Vector3d.VectorAngle(Vector3d.ZAxis, fieldVectors[i]) * (180.0 / Math.PI);
            }

            int iterations = 50;
            double anchoring = 1.0 - Clamp01(globalInfluence * 0.85);
            double smoothing = Clamp01(0.20 + (options.SmoothingFactor * 0.60));

            for (int iteration = 0; iteration < iterations; iteration++)
            {
                double[] next = new double[vertexCount];
                for (int i = 0; i < vertexCount; i++)
                {
                    List<int> neighbors = adjacency[i];
                    if (neighbors.Count == 0)
                    {
                        next[i] = baseField[i];
                        continue;
                    }

                    double compatibilityAccumulation = 0.0;
                    double compatibilityWeight = 0.0;
                    double laplacianAccumulation = 0.0;

                    foreach (int neighborIndex in neighbors)
                    {
                        Vector3d edge = vertices[neighborIndex] - vertices[i];
                        double edgeLength = edge.Length;
                        if (edgeLength <= tolerance)
                        {
                            continue;
                        }

                        Vector3d averageFieldVector = fieldVectors[i] + fieldVectors[neighborIndex];
                        if (!averageFieldVector.Unitize())
                        {
                            averageFieldVector = Vector3d.ZAxis;
                        }

                        double desiredDifference = (edge * averageFieldVector) / layerHeight;
                        double weight = 1.0 / edgeLength;
                        compatibilityAccumulation += (fieldValues[neighborIndex] - desiredDifference) * weight;
                        compatibilityWeight += weight;
                        laplacianAccumulation += fieldValues[neighborIndex];
                    }

                    if (compatibilityWeight <= 0.0)
                    {
                        next[i] = baseField[i];
                        continue;
                    }

                    double compatibleValue = compatibilityAccumulation / compatibilityWeight;
                    double smoothedValue = laplacianAccumulation / neighbors.Count;
                    double blendedValue = Lerp(compatibleValue, smoothedValue, smoothing);
                    double anchoredValue = Lerp(baseField[i], blendedValue, 1.0 - anchoring);

                    // Signed curvature slightly biases the field so high-curvature regions steer contours more coherently.
                    double curvatureBias = normalizedCurvature[i] * globalInfluence * Clamp01(options.SaddleBiasStrength) * 0.35;
                    next[i] = anchoredValue + curvatureBias;
                }

                fieldValues = next;
            }

            return new ScalarFieldData(fieldValues, fieldVectors, fieldAnglesDegrees, layerHeight);
        }

        private static Vector3d[] BuildDirectionTargets(Vector3d[] normals, Vector3d[] tangents, SliceGenerationOptions options)
        {
            int count = normals.Length;
            var targets = new Vector3d[count];

            for (int i = 0; i < count; i++)
            {
                Vector3d target = options.FieldMode == SurfaceFieldMode.SurfaceNormal ? normals[i] : tangents[i];
                if (!target.Unitize())
                {
                    target = Vector3d.ZAxis;
                }

                if (target * Vector3d.ZAxis < 0.0)
                {
                    target.Reverse();
                }

                targets[i] = target;
            }

            return targets;
        }

        private static Vector3d[] BuildTangentTargets(
            Point3d[] vertices,
            Vector3d[] normals,
            double[] normalizedCurvature,
            List<int>[] adjacency,
            SliceGenerationOptions options)
        {
            int vertexCount = vertices.Length;
            var targets = new Vector3d[vertexCount];

            for (int i = 0; i < vertexCount; i++)
            {
                Vector3d climbTangent = ProjectToTangentPlane(Vector3d.ZAxis, normals[i]);
                bool climbValid = climbTangent.Unitize();

                Vector3d curvatureTangent = EstimateCurvatureTangent(vertices, normals, normalizedCurvature, adjacency, i);
                bool curvatureValid = curvatureTangent.Unitize();

                double curvatureWeight = Math.Abs(normalizedCurvature[i]);
                double adaptiveBlend = Clamp01(options.AdaptiveBlendFactor) * curvatureWeight;

                if (climbValid && curvatureValid)
                {
                    targets[i] = BlendTangents(climbTangent, curvatureTangent, adaptiveBlend);
                    continue;
                }

                if (climbValid)
                {
                    targets[i] = climbTangent;
                    continue;
                }

                if (curvatureValid && curvatureWeight > 0.05)
                {
                    targets[i] = curvatureTangent;
                    continue;
                }

                targets[i] = Vector3d.ZAxis;
            }

            return targets;
        }

        private static Curve[] ExtractIsoCurves(Mesh mesh, IReadOnlyList<double> scalarValues, double level, double tolerance)
        {
            var segments = new List<LineCurve>();

            for (int faceIndex = 0; faceIndex < mesh.Faces.Count; faceIndex++)
            {
                MeshFace face = mesh.Faces[faceIndex];
                if (face.IsTriangle)
                {
                    AddTriangleIsoSegment(mesh, scalarValues, level, tolerance, face.A, face.B, face.C, segments);
                    continue;
                }

                AddTriangleIsoSegment(mesh, scalarValues, level, tolerance, face.A, face.B, face.C, segments);
                AddTriangleIsoSegment(mesh, scalarValues, level, tolerance, face.A, face.C, face.D, segments);
            }

            if (segments.Count == 0)
            {
                return Array.Empty<Curve>();
            }

            Curve[] joined = Curve.JoinCurves(segments.Cast<Curve>().ToArray(), tolerance * 4.0);
            if (joined.Length > 0)
            {
                return joined;
            }

            return segments.Cast<Curve>().ToArray();
        }

        private static Curve SmoothCurveOnMesh(Curve curve, Mesh mesh, SliceGenerationOptions options, double tolerance)
        {
            double smoothingFactor = Clamp01(options.OutputCurveSmoothingFactor);
            int iterations = Math.Max(0, options.OutputCurveSmoothingIterations);
            double allowedDeviation = Math.Max(tolerance, options.SurfaceFollowTolerance);

            if (smoothingFactor <= 0.0 || iterations == 0 || curve.GetLength() <= tolerance * 4.0)
            {
                return curve;
            }

            int divisionCount = Math.Max(12, (int)Math.Ceiling(curve.GetLength() / Math.Max(tolerance * 8.0, options.TargetLayerHeight * 0.35)));
            double[] parameters = curve.DivideByCount(divisionCount, true);
            if (parameters == null || parameters.Length < 4)
            {
                return curve;
            }

            bool closed = curve.IsClosed;
            var originalPoints = parameters.Select(curve.PointAt).ToList();
            var smoothedPoints = new List<Point3d>(originalPoints);

            for (int iteration = 0; iteration < iterations; iteration++)
            {
                var nextPoints = new List<Point3d>(smoothedPoints);
                for (int i = 0; i < smoothedPoints.Count; i++)
                {
                    bool fixedEndpoint = !closed && (i == 0 || i == smoothedPoints.Count - 1);
                    if (fixedEndpoint)
                    {
                        nextPoints[i] = originalPoints[i];
                        continue;
                    }

                    int previousIndex = i == 0 ? smoothedPoints.Count - 1 : i - 1;
                    int nextIndex = i == smoothedPoints.Count - 1 ? 0 : i + 1;

                    Point3d previous = smoothedPoints[previousIndex];
                    Point3d current = smoothedPoints[i];
                    Point3d next = smoothedPoints[nextIndex];
                    Point3d averaged = new Point3d(
                        (previous.X + next.X) * 0.5,
                        (previous.Y + next.Y) * 0.5,
                        (previous.Z + next.Z) * 0.5);

                    Point3d candidate = LerpPoint(current, averaged, smoothingFactor);
                    candidate = ProjectPointToMesh(mesh, candidate, allowedDeviation * 4.0, current);
                    candidate = ClampPointDeviation(originalPoints[i], candidate, allowedDeviation);
                    nextPoints[i] = candidate;
                }

                smoothedPoints = nextPoints;
            }

            if (closed && smoothedPoints.Count > 0)
            {
                smoothedPoints[smoothedPoints.Count - 1] = smoothedPoints[0];
            }

            Curve? smoothCurve = Curve.CreateInterpolatedCurve(smoothedPoints, 3);
            if (smoothCurve == null || !smoothCurve.IsValid)
            {
                return curve;
            }

            if (closed)
            {
                smoothCurve.MakeClosed(tolerance * 2.0);
            }

            return smoothCurve;
        }

        private static Curve ConvertCurveToOutputPolyline(Curve curve, Brep brep, Mesh mesh, SliceGenerationOptions options, double tolerance)
        {
            OutputPolylineSamplingMode samplingMode = options.OutputSamplingMode;
            bool closed = curve.IsClosed;
            int minimumPointCount = closed ? 3 : 2;

            if (samplingMode == OutputPolylineSamplingMode.None)
            {
                return curve;
            }

            double curveLength = curve.GetLength();
            if (curveLength <= tolerance)
            {
                return curve;
            }

            if (samplingMode == OutputPolylineSamplingMode.PointCount && options.OutputPolylinePointCount < minimumPointCount)
            {
                return curve;
            }

            if (samplingMode == OutputPolylineSamplingMode.MinimumDistance && options.OutputPolylineMinPointDistance <= tolerance)
            {
                return curve;
            }

            List<double> sampleLocations;
            if (samplingMode == OutputPolylineSamplingMode.PointCount)
            {
                sampleLocations = BuildFeatureAwareLocationsByCount(curve, options.OutputPolylinePointCount, tolerance);
            }
            else
            {
                sampleLocations = BuildFeatureAwareLocationsByDistance(curve, options.OutputPolylineMinPointDistance, tolerance);
            }

            if (sampleLocations.Count < minimumPointCount)
            {
                return curve;
            }

            var sampledPoints = new List<Point3d>();

            foreach (double normalizedLength in sampleLocations)
            {
                sampledPoints.Add(ProjectCurveSample(curve, normalizedLength, brep, mesh, options.SurfaceFollowTolerance, tolerance));
            }

            if (closed && sampledPoints.Count > 0)
            {
                sampledPoints.Add(sampledPoints[0]);
            }

            DeduplicateSequentialPoints(sampledPoints, tolerance);
            if (sampledPoints.Count < minimumPointCount)
            {
                return curve;
            }

            var polyline = new Polyline(sampledPoints);
            if (!polyline.IsValid || polyline.Count < minimumPointCount)
            {
                return curve;
            }

            return new PolylineCurve(polyline);
        }

        private static void AddTriangleIsoSegment(
            Mesh mesh,
            IReadOnlyList<double> scalarValues,
            double level,
            double tolerance,
            int a,
            int b,
            int c,
            List<LineCurve> segments)
        {
            var points = new List<Point3d>(3);
            TryAddIsoPoint(mesh, scalarValues, level, tolerance, a, b, points);
            TryAddIsoPoint(mesh, scalarValues, level, tolerance, b, c, points);
            TryAddIsoPoint(mesh, scalarValues, level, tolerance, c, a, points);
            DeduplicatePoints(points, tolerance * 2.0);

            if (points.Count == 2)
            {
                if (points[0].DistanceToSquared(points[1]) > tolerance * tolerance)
                {
                    segments.Add(new LineCurve(points[0], points[1]));
                }
            }
            else if (points.Count > 2)
            {
                points = points
                    .OrderBy(point => point.X)
                    .ThenBy(point => point.Y)
                    .ThenBy(point => point.Z)
                    .ToList();

                int lastIndex = points.Count - 1;
                if (points[0].DistanceToSquared(points[lastIndex]) > tolerance * tolerance)
                {
                    segments.Add(new LineCurve(points[0], points[lastIndex]));
                }
            }
        }

        private static void TryAddIsoPoint(
            Mesh mesh,
            IReadOnlyList<double> scalarValues,
            double level,
            double tolerance,
            int startIndex,
            int endIndex,
            List<Point3d> points)
        {
            double startValue = scalarValues[startIndex];
            double endValue = scalarValues[endIndex];
            double startDelta = startValue - level;
            double endDelta = endValue - level;
            double epsilon = 1e-8;

            if (Math.Abs(startDelta) <= epsilon && Math.Abs(endDelta) <= epsilon)
            {
                points.Add(mesh.Vertices.Point3dAt(startIndex));
                points.Add(mesh.Vertices.Point3dAt(endIndex));
                return;
            }

            if ((startDelta < -epsilon && endDelta < -epsilon) || (startDelta > epsilon && endDelta > epsilon))
            {
                return;
            }

            Point3d start = mesh.Vertices.Point3dAt(startIndex);
            Point3d end = mesh.Vertices.Point3dAt(endIndex);

            if (Math.Abs(startDelta) <= epsilon)
            {
                points.Add(start);
                return;
            }

            if (Math.Abs(endDelta) <= epsilon)
            {
                points.Add(end);
                return;
            }

            double denominator = endValue - startValue;
            if (Math.Abs(denominator) <= epsilon)
            {
                return;
            }

            double t = (level - startValue) / denominator;
            t = Math.Max(0.0, Math.Min(1.0, t));

            if (t <= epsilon || t >= 1.0 - epsilon)
            {
                points.Add(t <= 0.5 ? start : end);
                return;
            }

            points.Add(start + ((end - start) * t));
        }

        private static Plane BuildRepresentativePlane(Curve curve, Mesh mesh, IReadOnlyList<Vector3d> fieldVectors, double tolerance)
        {
            double middleParameter = curve.Domain.ParameterAt(0.5);
            double halfLength = curve.GetLength() * 0.5;
            if (!curve.LengthParameter(halfLength, out middleParameter))
            {
                middleParameter = curve.Domain.ParameterAt(0.5);
            }

            Point3d origin = curve.PointAt(middleParameter);
            Vector3d tangent = curve.TangentAt(middleParameter);
            if (!tangent.Unitize())
            {
                tangent = Vector3d.XAxis;
            }

            Vector3d fieldNormal = AverageFieldVectorAlongCurve(curve, mesh, fieldVectors, tolerance);
            if (Math.Abs(fieldNormal * tangent) > 0.98)
            {
                fieldNormal = Vector3d.ZAxis;
            }

            Vector3d yAxis = Vector3d.CrossProduct(fieldNormal, tangent);
            if (!yAxis.Unitize())
            {
                yAxis = Vector3d.CrossProduct(Vector3d.ZAxis, tangent);
                if (!yAxis.Unitize())
                {
                    yAxis = Vector3d.YAxis;
                }
            }

            return new Plane(origin, tangent, yAxis);
        }

        private static List<double> BuildFeatureAwareLocationsByCount(Curve curve, int requestedPointCount, double tolerance)
        {
            bool closed = curve.IsClosed;
            int minimumPointCount = closed ? 3 : 2;
            int targetCount = Math.Max(requestedPointCount, minimumPointCount);
            if (targetCount < minimumPointCount)
            {
                return new List<double>();
            }

            double baseSpacing = closed
                ? 1.0 / targetCount
                : 1.0 / Math.Max(1, targetCount - 1);
            double spacingHint = curve.GetLength() * baseSpacing;

            List<FeatureLocation> anchors = CollectFeatureLocations(curve, spacingHint, tolerance);
            var selected = new List<double>();

            foreach (FeatureLocation anchor in anchors.OrderByDescending(location => location.Score))
            {
                if (selected.Count >= targetCount)
                {
                    break;
                }

                if (ContainsNormalizedLocationNear(selected, anchor.NormalizedLength, baseSpacing * 0.5, closed))
                {
                    continue;
                }

                selected.Add(anchor.NormalizedLength);
            }

            while (selected.Count < targetCount)
            {
                double nextLocation = FindLargestGapMidpoint(selected, closed);
                if (ContainsNormalizedLocationNear(selected, nextLocation, baseSpacing * 0.1, closed))
                {
                    break;
                }

                selected.Add(nextLocation);
            }

            return selected
                .OrderBy(value => value)
                .ToList();
        }

        private static List<double> BuildFeatureAwareLocationsByDistance(Curve curve, double minimumPointDistance, double tolerance)
        {
            bool closed = curve.IsClosed;
            int minimumPointCount = closed ? 3 : 2;
            double curveLength = curve.GetLength();
            if (curveLength <= tolerance)
            {
                return new List<double>();
            }

            double normalizedSpacing = Math.Min(1.0, Math.Max(tolerance / curveLength, minimumPointDistance / curveLength));

            List<FeatureLocation> anchors = CollectFeatureLocations(curve, minimumPointDistance, tolerance);

            var selected = new List<double>();

            foreach (FeatureLocation anchor in anchors.OrderByDescending(location => location.Score))
            {
                if (ContainsNormalizedLocationNear(selected, anchor.NormalizedLength, normalizedSpacing, closed))
                {
                    continue;
                }

                selected.Add(anchor.NormalizedLength);
            }

            selected = selected
                .OrderBy(value => value)
                .ToList();

            if (selected.Count < minimumPointCount) 
            {
                if (closed && selected.Count == 0)
                {
                    selected.Add(0.0);
                }
                else if (!closed)
                {
                    if (!ContainsNormalizedLocationNear(selected, 0.0, normalizedSpacing * 0.25, false))
                    {
                        selected.Add(0.0);
                    }

                    if (!ContainsNormalizedLocationNear(selected, 1.0, normalizedSpacing * 0.25, false))
                    {
                        selected.Add(1.0);
                    }
                }
            }

            selected = selected
                .OrderBy(value => value)
                .ToList();

            selected = FillGapsWithMinimumSpacing(selected, normalizedSpacing, closed);

            return selected;
        }

        private static List<FeatureLocation> CollectFeatureLocations(Curve curve, double spacingHint, double tolerance)
        {
            bool closed = curve.IsClosed;
            var anchors = new List<FeatureLocation>();
            if (closed)
            {
                anchors.Add(new FeatureLocation(0.0, double.MaxValue));
            }
            else
            {
                anchors.Add(new FeatureLocation(0.0, double.MaxValue));
                anchors.Add(new FeatureLocation(1.0, double.MaxValue));
            }

            anchors.AddRange(CollectPolylineFeatureParameters(curve, tolerance));
            anchors.AddRange(CollectDenseFeatureParameters(curve, spacingHint, tolerance));
            return anchors;
        }

        private static IEnumerable<FeatureLocation> CollectPolylineFeatureParameters(Curve curve, double tolerance)
        {
            var features = new List<FeatureLocation>();
            if (!curve.TryGetPolyline(out Polyline polyline) || polyline.Count < 3)
            {
                return features;
            }

            bool closed = curve.IsClosed;
            int lastUsableIndex = closed ? polyline.Count - 1 : polyline.Count;

            for (int i = 0; i < lastUsableIndex; i++)
            {
                bool endpoint = !closed && (i == 0 || i == lastUsableIndex - 1);
                if (endpoint)
                {
                    continue;
                }

                int previousIndex = i == 0 ? lastUsableIndex - 1 : i - 1;
                int nextIndex = i == lastUsableIndex - 1 ? 0 : i + 1;
                double turnAngle = ComputeTurnAngle(polyline[previousIndex], polyline[i], polyline[nextIndex]);
                if (turnAngle < 8.0)
                {
                    continue;
                }

                double parameter;
                if (!curve.ClosestPoint(polyline[i], out parameter, tolerance * 4.0))
                {
                    continue;
                }

                features.Add(new FeatureLocation(GetNormalizedLengthAtParameter(curve, parameter), 1000.0 + turnAngle));
            }

            return features;
        }

        private static IEnumerable<FeatureLocation> CollectDenseFeatureParameters(Curve curve, double spacingHint, double tolerance)
        {
            var features = new List<FeatureLocation>();
            bool closed = curve.IsClosed;
            double curveLength = curve.GetLength();
            int denseCount = Math.Max((int)Math.Ceiling(curveLength / Math.Max(spacingHint * 0.35, tolerance * 6.0)), closed ? 48 : 32);
            List<double> parameters = SampleCurveParameters(curve, denseCount, closed);
            List<Point3d> points = parameters.Select(curve.PointAt).ToList();

            int count = points.Count;
            for (int i = 0; i < count; i++)
            {
                bool endpoint = !closed && (i == 0 || i == count - 1);
                if (endpoint)
                {
                    continue;
                }

                int previousIndex = i == 0 ? count - 1 : i - 1;
                int nextIndex = i == count - 1 ? 0 : i + 1;
                double turnAngle = ComputeTurnAngle(points[previousIndex], points[i], points[nextIndex]);
                if (turnAngle < 10.0)
                {
                    continue;
                }

                double previousTurn = ComputeTurnAngle(points[previousIndex == 0 ? count - 1 : previousIndex - 1], points[previousIndex], points[i]);
                double nextTurn = ComputeTurnAngle(points[i], points[nextIndex], points[nextIndex == count - 1 ? 0 : nextIndex + 1]);
                if (turnAngle + 1e-6 < previousTurn || turnAngle + 1e-6 < nextTurn)
                {
                    continue;
                }

                features.Add(new FeatureLocation(GetNormalizedLengthAtParameter(curve, parameters[i]), turnAngle));
            }

            return features;
        }

        private static List<double> SampleCurveParameters(Curve curve, int count, bool closed)
        {
            var parameters = new List<double>(count);
            if (closed)
            {
                for (int i = 0; i < count; i++)
                {
                    double normalized = (double)i / count;
                    parameters.Add(curve.Domain.ParameterAt(normalized));
                }
            }
            else
            {
                for (int i = 0; i < count; i++)
                {
                    double normalized = count == 1 ? 0.0 : (double)i / (count - 1);
                    parameters.Add(curve.Domain.ParameterAt(normalized));
                }
            }

            return parameters;
        }

        private static double ComputeTurnAngle(Point3d previous, Point3d current, Point3d next)
        {
            Vector3d incoming = current - previous;
            Vector3d outgoing = next - current;
            if (!incoming.Unitize() || !outgoing.Unitize())
            {
                return 0.0;
            }

            return Vector3d.VectorAngle(incoming, outgoing) * (180.0 / Math.PI);
        }

        private static bool ContainsNormalizedLocationNear(IReadOnlyList<double> locations, double candidate, double normalizedSpacing, bool closed)
        {
            foreach (double location in locations)
            {
                double distance = Math.Abs(location - candidate);
                if (distance <= normalizedSpacing)
                {
                    return true;
                }

                if (closed)
                {
                    double wrappedDistance = Math.Min(distance, 1.0 - distance);
                    if (wrappedDistance <= normalizedSpacing)
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        private static double FindLargestGapMidpoint(IReadOnlyList<double> selectedLocations, bool closed)
        {
            if (selectedLocations.Count == 0)
            {
                return 0.5;
            }

            List<double> sorted = selectedLocations
                .OrderBy(value => value)
                .ToList();

            double bestGap = double.MinValue;
            double midpoint = 0.5;

            for (int i = 0; i < sorted.Count - 1; i++)
            {
                double gap = sorted[i + 1] - sorted[i];
                if (gap > bestGap)
                {
                    bestGap = gap;
                    midpoint = sorted[i] + (0.5 * gap);
                }
            }

            if (closed)
            {
                double wrapGap = (sorted[0] + 1.0) - sorted[sorted.Count - 1];
                if (wrapGap > bestGap)
                {
                    midpoint = sorted[sorted.Count - 1] + (0.5 * wrapGap);
                    if (midpoint >= 1.0)
                    {
                        midpoint -= 1.0;
                    }
                }
            }
            else
            {
                if (sorted[0] > bestGap)
                {
                    bestGap = sorted[0];
                    midpoint = 0.5 * sorted[0];
                }

                double tailGap = 1.0 - sorted[sorted.Count - 1];
                if (tailGap > bestGap)
                {
                    midpoint = sorted[sorted.Count - 1] + (0.5 * tailGap);
                }
            }

            return midpoint;
        }

        private static List<double> FillGapsWithMinimumSpacing(IReadOnlyList<double> selectedLocations, double normalizedSpacing, bool closed)
        {
            if (selectedLocations.Count == 0)
            {
                return new List<double>();
            }

            List<double> sorted = selectedLocations
                .OrderBy(value => value)
                .ToList();
            var filled = new List<double>();

            if (!closed)
            {
                for (int i = 0; i < sorted.Count - 1; i++)
                {
                    double start = sorted[i];
                    double end = sorted[i + 1];
                    if (i == 0)
                    {
                        filled.Add(start);
                    }

                    AppendIntermediateLocations(filled, start, end, normalizedSpacing);
                }

                filled.Add(sorted[sorted.Count - 1]);
            }
            else
            {
                for (int i = 0; i < sorted.Count; i++)
                {
                    double start = sorted[i];
                    double end = i == sorted.Count - 1 ? sorted[0] + 1.0 : sorted[i + 1];
                    if (i == 0)
                    {
                        filled.Add(start);
                    }

                    AppendIntermediateLocations(filled, start, end, normalizedSpacing);
                }

                for (int i = 0; i < filled.Count; i++)
                {
                    if (filled[i] >= 1.0)
                    {
                        filled[i] -= 1.0;
                    }
                }
            }

            return filled
                .Distinct()
                .OrderBy(value => value)
                .ToList();
        }

        private static void AppendIntermediateLocations(List<double> filled, double start, double end, double normalizedSpacing)
        {
            if (filled.Count == 0)
            {
                filled.Add(start);
            }

            double gap = end - start;
            int segments = Math.Max(1, (int)Math.Floor(gap / normalizedSpacing));
            double step = gap / segments;

            for (int i = 1; i <= segments; i++)
            {
                double sample = start + (i * step);
                if (i == segments && Math.Abs(sample - end) <= 1e-9)
                {
                    continue;
                }

                filled.Add(sample);
            }
        }

        private static double GetNormalizedLengthAtParameter(Curve curve, double parameter)
        {
            double totalLength = curve.GetLength();
            if (totalLength <= 1e-9)
            {
                return 0.0;
            }

            if (parameter <= curve.Domain.T0)
            {
                return 0.0;
            }

            if (parameter >= curve.Domain.T1)
            {
                return 1.0;
            }

            Interval subDomain = new Interval(curve.Domain.T0, parameter);
            double partialLength = curve.GetLength(subDomain);
            if (partialLength <= 0.0)
            {
                return 0.0;
            }

            return Math.Max(0.0, Math.Min(1.0, partialLength / totalLength));
        }

        private static Point3d ProjectCurveSample(Curve curve, double normalizedLength, Brep brep, Mesh mesh, double surfaceFollowTolerance, double tolerance)
        {
            double curveParameter = curve.Domain.ParameterAt(normalizedLength);
            if (!curve.NormalizedLengthParameter(normalizedLength, out curveParameter))
            {
                curveParameter = curve.Domain.ParameterAt(normalizedLength);
            }

            Point3d samplePoint = curve.PointAt(curveParameter);
            Point3d projectedToBrep = ProjectPointToBrep(brep, samplePoint, tolerance);
            if (projectedToBrep.DistanceTo(samplePoint) <= Math.Max(surfaceFollowTolerance, tolerance))
            {
                return projectedToBrep;
            }

            Point3d projectedToMesh = ProjectPointToMesh(mesh, samplePoint, Math.Max(surfaceFollowTolerance, tolerance) * 4.0, samplePoint);
            Point3d clampedToBrep = ProjectPointToBrep(brep, projectedToMesh, tolerance);
            return ClampPointDeviation(samplePoint, clampedToBrep, Math.Max(surfaceFollowTolerance, tolerance));
        }

        private static Vector3d AverageFieldVectorAlongCurve(Curve curve, Mesh mesh, IReadOnlyList<Vector3d> fieldVectors, double tolerance)
        {
            var average = Vector3d.Zero;
            double[]? parameters = curve.DivideByCount(12, true);
            if (parameters == null || parameters.Length == 0)
            {
                parameters = new[] { curve.Domain.ParameterAt(0.5) };
            }

            foreach (double parameter in parameters)
            {
                Point3d point = curve.PointAt(parameter);
                MeshPoint? meshPoint = mesh.ClosestMeshPoint(point, Math.Max(tolerance * 10.0, 1.0));
                if (meshPoint == null || meshPoint.FaceIndex < 0 || meshPoint.FaceIndex >= mesh.Faces.Count)
                {
                    continue;
                }

                MeshFace face = mesh.Faces[meshPoint.FaceIndex];
                average += fieldVectors[face.A];
                average += fieldVectors[face.B];
                average += fieldVectors[face.C];
                if (!face.IsTriangle)
                {
                    average += fieldVectors[face.D];
                }
            }

            if (!average.Unitize())
            {
                return Vector3d.ZAxis;
            }

            if (average * Vector3d.ZAxis < 0.0)
            {
                average.Reverse();
            }

            return average;
        }

        private static Vector3d EstimateCurvatureTangent(
            Point3d[] vertices,
            Vector3d[] normals,
            IReadOnlyList<double> normalizedCurvature,
            List<int>[] adjacency,
            int vertexIndex)
        {
            Vector3d normal = normals[vertexIndex];
            Vector3d bestDirection = Vector3d.Zero;
            double bestScore = double.MinValue;

            foreach (int neighborIndex in adjacency[vertexIndex])
            {
                Vector3d edge = vertices[neighborIndex] - vertices[vertexIndex];
                double edgeLength = edge.Length;
                if (edgeLength <= 1e-9)
                {
                    continue;
                }

                Vector3d tangentEdge = ProjectToTangentPlane(edge, normal);
                if (!tangentEdge.Unitize())
                {
                    continue;
                }

                double normalChange = (normals[neighborIndex] - normals[vertexIndex]).Length / edgeLength;
                double curvatureChange = Math.Abs(normalizedCurvature[neighborIndex] - normalizedCurvature[vertexIndex]);
                double score = normalChange + (0.5 * curvatureChange);
                if (score > bestScore)
                {
                    bestScore = score;
                    bestDirection = tangentEdge;
                }
            }

            if (bestDirection.IsTiny())
            {
                bestDirection = ProjectToTangentPlane(Vector3d.XAxis, normal);
                if (!bestDirection.Unitize())
                {
                    bestDirection = ProjectToTangentPlane(Vector3d.YAxis, normal);
                }
            }

            if (!bestDirection.Unitize())
            {
                return Vector3d.Zero;
            }

            if (bestDirection * Vector3d.ZAxis < 0.0)
            {
                bestDirection.Reverse();
            }

            return bestDirection;
        }

        private static List<int>[] BuildAdjacency(Mesh mesh)
        {
            var adjacency = Enumerable.Range(0, mesh.Vertices.Count)
                .Select(_ => new List<int>())
                .ToArray();

            for (int faceIndex = 0; faceIndex < mesh.Faces.Count; faceIndex++)
            {
                MeshFace face = mesh.Faces[faceIndex];
                AddUndirectedEdge(adjacency, face.A, face.B);
                AddUndirectedEdge(adjacency, face.B, face.C);
                AddUndirectedEdge(adjacency, face.C, face.A);

                if (!face.IsTriangle)
                {
                    AddUndirectedEdge(adjacency, face.C, face.D);
                    AddUndirectedEdge(adjacency, face.D, face.A);
                }
            }

            return adjacency;
        }

        private static void AddUndirectedEdge(List<int>[] adjacency, int a, int b)
        {
            if (a == b)
            {
                return;
            }

            if (!adjacency[a].Contains(b))
            {
                adjacency[a].Add(b);
            }

            if (!adjacency[b].Contains(a))
            {
                adjacency[b].Add(a);
            }
        }

        private static Vector3d[] BuildVertexNormals(Mesh mesh, List<int>[] adjacency, SliceGenerationOptions options)
        {
            int vertexCount = mesh.Vertices.Count;
            var normals = new Vector3d[vertexCount];

            for (int i = 0; i < vertexCount; i++)
            {
                Vector3f normal = mesh.Normals[i];
                normals[i] = new Vector3d(normal.X, normal.Y, normal.Z);
                if (!normals[i].Unitize())
                {
                    normals[i] = Vector3d.ZAxis;
                }

                if (normals[i] * Vector3d.ZAxis < 0.0)
                {
                    normals[i].Reverse();
                }
            }

            int smoothingIterations = 3;
            double smoothing = Clamp01(options.SmoothingFactor);
            for (int iteration = 0; iteration < smoothingIterations; iteration++)
            {
                var next = new Vector3d[vertexCount];
                for (int i = 0; i < vertexCount; i++)
                {
                    if (adjacency[i].Count == 0)
                    {
                        next[i] = normals[i];
                        continue;
                    }

                    Vector3d averaged = normals[i];
                    foreach (int neighborIndex in adjacency[i])
                    {
                        averaged += normals[neighborIndex];
                    }

                    if (!averaged.Unitize())
                    {
                        averaged = normals[i];
                    }

                    next[i] = BlendVectors(normals[i], averaged, smoothing * 0.5);
                    if (next[i] * Vector3d.ZAxis < 0.0)
                    {
                        next[i].Reverse();
                    }
                }

                normals = next;
            }

            return normals;
        }

        private static double[] BuildSignedCurvature(Point3d[] vertices, Vector3d[] normals, List<int>[] adjacency)
        {
            var curvature = new double[vertices.Length];

            for (int i = 0; i < vertices.Length; i++)
            {
                if (adjacency[i].Count == 0)
                {
                    curvature[i] = 0.0;
                    continue;
                }

                Vector3d laplacian = Vector3d.Zero;
                double averageLength = 0.0;

                foreach (int neighborIndex in adjacency[i])
                {
                    Vector3d edge = vertices[neighborIndex] - vertices[i];
                    averageLength += edge.Length;
                    laplacian += edge;
                }

                averageLength /= adjacency[i].Count;
                if (averageLength <= 1e-9)
                {
                    curvature[i] = 0.0;
                    continue;
                }

                laplacian /= adjacency[i].Count;
                curvature[i] = (laplacian * normals[i]) / averageLength;
            }

            return curvature;
        }

        private static double[] NormalizeSignedValues(IReadOnlyList<double> values)
        {
            double maxAbsolute = values.Select(Math.Abs).DefaultIfEmpty(0.0).Max();
            if (maxAbsolute <= 1e-9)
            {
                return values.Select(_ => 0.0).ToArray();
            }

            return values.Select(value => value / maxAbsolute).ToArray();
        }

        private static Vector3d BlendVectors(Vector3d start, Vector3d end, double t)
        {
            double clamped = Clamp01(t);
            Vector3d blended = new Vector3d(
                start.X + ((end.X - start.X) * clamped),
                start.Y + ((end.Y - start.Y) * clamped),
                start.Z + ((end.Z - start.Z) * clamped));

            if (!blended.Unitize())
            {
                return Vector3d.ZAxis;
            }

            if (blended * Vector3d.ZAxis < 0.0)
            {
                blended.Reverse();
            }

            return blended;
        }

        private static Vector3d BlendTangents(Vector3d a, Vector3d b, double t)
        {
            if (a.IsTiny())
            {
                return b.IsTiny() ? Vector3d.ZAxis : b;
            }

            if (b.IsTiny())
            {
                return a;
            }

            if ((a * b) < 0.0)
            {
                b.Reverse();
            }

            return BlendVectors(a, b, t);
        }

        private static Vector3d BuildFieldVector(Vector3d targetDirection, double normalizedInfluence)
        {
            if (normalizedInfluence <= 0.0)
            {
                return Vector3d.ZAxis;
            }

            Vector3d target = targetDirection;
            if (!target.Unitize())
            {
                return Vector3d.ZAxis;
            }

            if (target * Vector3d.ZAxis < 0.0)
            {
                target.Reverse();
            }

            double rawAngle = Vector3d.VectorAngle(Vector3d.ZAxis, target) * (180.0 / Math.PI);
            if (rawAngle <= 1e-9)
            {
                return Vector3d.ZAxis;
            }

            double desiredAngle = rawAngle * Clamp01(normalizedInfluence);
            if (desiredAngle <= 1e-9)
            {
                return Vector3d.ZAxis;
            }

            Vector3d lateral = ProjectToTangentPlane(target, Vector3d.ZAxis);
            if (!lateral.Unitize())
            {
                return Vector3d.ZAxis;
            }

            double radians = desiredAngle * (Math.PI / 180.0);
            Vector3d result = (Vector3d.ZAxis * Math.Cos(radians)) + (lateral * Math.Sin(radians));
            if (!result.Unitize())
            {
                return Vector3d.ZAxis;
            }

            return result;
        }

        private static Mesh CreateAnglePreviewMesh(Mesh sourceMesh, IReadOnlyList<double> anglesDegrees)
        {
            Mesh previewMesh = sourceMesh.DuplicateMesh();
            previewMesh.VertexColors.Clear();

            double displayMax = Math.Max(1.0, anglesDegrees.DefaultIfEmpty(0.0).Max());
            for (int i = 0; i < previewMesh.Vertices.Count; i++)
            {
                double angle = i < anglesDegrees.Count ? anglesDegrees[i] : 0.0;
                previewMesh.VertexColors.Add(MapAngleToColor(angle, displayMax));
            }

            return previewMesh;
        }

        private static List<Line> CreateFieldLines(Mesh mesh, IReadOnlyList<Vector3d> fieldVectors, double layerHeight)
        {
            var lines = new List<Line>();
            int maxGlyphCount = 300;
            int step = Math.Max(1, mesh.Vertices.Count / maxGlyphCount);
            double glyphLength = Math.Max(layerHeight * 0.6, 1.0);

            for (int i = 0; i < mesh.Vertices.Count; i += step)
            {
                Point3d origin = mesh.Vertices.Point3dAt(i);
                Vector3d vector = i < fieldVectors.Count ? fieldVectors[i] : Vector3d.ZAxis;
                if (!vector.Unitize())
                {
                    continue;
                }

                lines.Add(new Line(origin, origin + (vector * glyphLength)));
            }

            return lines;
        }

        private static string BuildAngleHistogram(IReadOnlyList<double> anglesDegrees)
        {
            if (anglesDegrees.Count == 0)
            {
                return "No angle samples available.";
            }

            double histogramMax = Math.Max(5.0, anglesDegrees.Max());
            int binCount = 12;
            double binSize = histogramMax / binCount;
            int[] counts = new int[binCount];

            foreach (double angle in anglesDegrees)
            {
                int index = Math.Min(binCount - 1, (int)Math.Floor(angle / binSize));
                counts[index]++;
            }

            int total = anglesDegrees.Count;
            var lines = new List<string>
            {
                $"Angle Histogram -> samples: {total}, range: 0.0 to {histogramMax:F1} deg"
            };

            for (int i = 0; i < binCount; i++)
            {
                double start = i * binSize;
                double end = (i + 1) * binSize;
                double percent = total <= 0 ? 0.0 : (100.0 * counts[i] / total);
                lines.Add($"{start,6:F1} - {end,6:F1} deg : {counts[i],5} ({percent,5:F1}%)");
            }

            return string.Join(Environment.NewLine, lines);
        }

        private static Color MapAngleToColor(double angleDegrees, double maxAngleDegrees)
        {
            double t = Clamp01(angleDegrees / Math.Max(1.0, maxAngleDegrees));
            if (t <= 0.5)
            {
                return InterpolateColor(Color.FromArgb(60, 120, 255), Color.FromArgb(255, 220, 80), t * 2.0);
            }

            return InterpolateColor(Color.FromArgb(255, 220, 80), Color.FromArgb(220, 70, 40), (t - 0.5) * 2.0);
        }

        private static Color InterpolateColor(Color start, Color end, double t)
        {
            double clamped = Clamp01(t);
            int r = (int)Math.Round(start.R + ((end.R - start.R) * clamped));
            int g = (int)Math.Round(start.G + ((end.G - start.G) * clamped));
            int b = (int)Math.Round(start.B + ((end.B - start.B) * clamped));
            return Color.FromArgb(r, g, b);
        }

        private static Vector3d ProjectToTangentPlane(Vector3d vector, Vector3d normal)
        {
            Vector3d unitNormal = normal;
            if (!unitNormal.Unitize())
            {
                return Vector3d.Zero;
            }

            return vector - ((vector * unitNormal) * unitNormal);
        }

        private static Point3d ProjectPointToMesh(Mesh mesh, Point3d point, double searchDistance, Point3d fallback)
        {
            MeshPoint meshPoint = mesh.ClosestMeshPoint(point, Math.Max(searchDistance, 1.0));
            if (meshPoint == null)
            {
                return fallback;
            }

            return meshPoint.Point;
        }

        private static Point3d ProjectPointToBrep(Brep brep, Point3d point, double tolerance)
        {
            Point3d closestPoint;
            ComponentIndex componentIndex;
            double s;
            double t;
            Vector3d normal;

            if (brep.ClosestPoint(point, out closestPoint, out componentIndex, out s, out t, tolerance, out normal))
            {
                return closestPoint;
            }

            return point;
        }

        private static Point3d ClampPointDeviation(Point3d originalPoint, Point3d candidatePoint, double maxDeviation)
        {
            Vector3d deviation = candidatePoint - originalPoint;
            double length = deviation.Length;
            if (length <= maxDeviation || length <= 1e-9)
            {
                return candidatePoint;
            }

            deviation.Unitize();
            return originalPoint + (deviation * maxDeviation);
        }

        private static void DeduplicateSequentialPoints(List<Point3d> points, double tolerance)
        {
            for (int i = points.Count - 1; i > 0; i--)
            {
                if (points[i].DistanceToSquared(points[i - 1]) <= tolerance * tolerance)
                {
                    points.RemoveAt(i);
                }
            }
        }

        private static void DeduplicatePoints(List<Point3d> points, double tolerance)
        {
            for (int i = points.Count - 1; i >= 0; i--)
            {
                for (int j = i - 1; j >= 0; j--)
                {
                    if (points[i].DistanceToSquared(points[j]) <= tolerance * tolerance)
                    {
                        points.RemoveAt(i);
                        break;
                    }
                }
            }
        }

        private static double Clamp01(double value)
        {
            if (value < 0.0)
            {
                return 0.0;
            }

            return value > 1.0 ? 1.0 : value;
        }

        private static double Lerp(double start, double end, double t)
        {
            return start + ((end - start) * Clamp01(t));
        }

        private static Point3d LerpPoint(Point3d start, Point3d end, double t)
        {
            double clamped = Clamp01(t);
            return new Point3d(
                start.X + ((end.X - start.X) * clamped),
                start.Y + ((end.Y - start.Y) * clamped),
                start.Z + ((end.Z - start.Z) * clamped));
        }

        private sealed class ScalarFieldData
        {
            public ScalarFieldData(double[] values, Vector3d[] fieldVectors, double[] anglesDegrees, double layerHeight)
            {
                Values = values;
                FieldVectors = fieldVectors;
                AnglesDegrees = anglesDegrees;
                LayerHeight = layerHeight;
            }

            public double[] Values { get; }
            public Vector3d[] FieldVectors { get; }
            public double[] AnglesDegrees { get; }
            public double LayerHeight { get; }
        }

        private sealed class FeatureLocation
        {
            public FeatureLocation(double normalizedLength, double score)
            {
                NormalizedLength = normalizedLength;
                Score = score;
            }

            public double NormalizedLength { get; }
            public double Score { get; }
        }
    }

    internal static class RhinoSliceAnalysis
    {
        public static string Format(IReadOnlyList<Curve> curves, IReadOnlyList<Plane> planes, double layerHeight, IReadOnlyList<double> scalarValues)
        {
            List<double> segmentLengths = ExtractSegmentLengths(curves);
            List<double> xAngles = planes.Select(plane => Vector3d.VectorAngle(plane.Normal, Vector3d.XAxis) * (180.0 / Math.PI)).ToList();
            List<double> yAngles = planes.Select(plane => Vector3d.VectorAngle(plane.Normal, Vector3d.YAxis) * (180.0 / Math.PI)).ToList();
            List<double> zAngles = planes.Select(plane => Vector3d.VectorAngle(plane.Normal, Vector3d.ZAxis) * (180.0 / Math.PI)).ToList();
            double scalarMin = scalarValues.DefaultIfEmpty(0.0).Min();
            double scalarMax = scalarValues.DefaultIfEmpty(0.0).Max();

            return
                $"Layers -> curves: {curves.Count}, spacing: {layerHeight:F3}, scalar range: {scalarMin:F3} to {scalarMax:F3}{Environment.NewLine}" +
                $"Segment Lengths -> min: {segmentLengths.DefaultIfEmpty(0.0).Min():F3}, max: {segmentLengths.DefaultIfEmpty(0.0).Max():F3}, avg: {segmentLengths.DefaultIfEmpty(0.0).Average():F3}{Environment.NewLine}" +
                $"Plane Angle From Global X -> min: {xAngles.DefaultIfEmpty(0.0).Min():F3}, max: {xAngles.DefaultIfEmpty(0.0).Max():F3}, avg: {xAngles.DefaultIfEmpty(0.0).Average():F3}{Environment.NewLine}" +
                $"Plane Angle From Global Y -> min: {yAngles.DefaultIfEmpty(0.0).Min():F3}, max: {yAngles.DefaultIfEmpty(0.0).Max():F3}, avg: {yAngles.DefaultIfEmpty(0.0).Average():F3}{Environment.NewLine}" +
                $"Plane Angle From Global Z -> min: {zAngles.DefaultIfEmpty(0.0).Min():F3}, max: {zAngles.DefaultIfEmpty(0.0).Max():F3}, avg: {zAngles.DefaultIfEmpty(0.0).Average():F3}";
        }

        private static List<double> ExtractSegmentLengths(IReadOnlyList<Curve> curves)
        {
            var lengths = new List<double>();
            foreach (Curve curve in curves)
            {
                if (curve.TryGetPolyline(out Polyline polyline))
                {
                    for (int i = 0; i < polyline.SegmentCount; i++)
                    {
                        lengths.Add(polyline.SegmentAt(i).Length);
                    }

                    continue;
                }

                double[]? parameters = curve.DivideByCount(Math.Max(2, (int)Math.Ceiling(curve.GetLength() / 1.0)), true);
                if (parameters == null || parameters.Length < 2)
                {
                    continue;
                }

                for (int i = 0; i < parameters.Length - 1; i++)
                {
                    lengths.Add(curve.PointAt(parameters[i]).DistanceTo(curve.PointAt(parameters[i + 1])));
                }
            }

            return lengths;
        }
    }
}
