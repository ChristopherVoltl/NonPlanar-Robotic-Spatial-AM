using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using SpatialAdditiveManufacturing.Core.Geometry;
using SpatialAdditiveManufacturing.Core.Slicing;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Bundles the Rhino geometry and diagnostics produced by the S3-inspired slicer.
    /// </summary>
    /// <remarks>
    /// This wrapper exists because the Grasshopper component needs both the slice curves and the field diagnostics that explain them.
    /// The class is immutable after construction.
    /// </remarks>
    internal sealed class RhinoS3SliceResult
    {
        /// <summary>
        /// Initializes a Rhino-side S3 slice result.
        /// </summary>
        /// <remarks>
        /// Preconditions: arguments should not be <see langword="null"/> and should all correspond to the same solve.
        /// Postconditions: references are stored exactly as supplied.
        /// Exceptions: none in this constructor.
        /// Side-effects: none.
        /// </remarks>
        public RhinoS3SliceResult(
            IReadOnlyList<Curve> curves,
            IReadOnlyList<Plane> planes,
            string analysis,
            Mesh fieldPreviewMesh,
            IReadOnlyList<Line> fieldLines,
            string fieldHistogram)
        {
            Curves = curves;
            Planes = planes;
            Analysis = analysis;
            FieldPreviewMesh = fieldPreviewMesh;
            FieldLines = fieldLines;
            FieldHistogram = fieldHistogram;
        }

        public IReadOnlyList<Curve> Curves { get; }
        public IReadOnlyList<Plane> Planes { get; }
        public string Analysis { get; }
        public Mesh FieldPreviewMesh { get; }
        public IReadOnlyList<Line> FieldLines { get; }
        public string FieldHistogram { get; }
    }

    /// <summary>
    /// Rhino-side adapter that converts a Brep into an S3-inspired field solve and extracted iso-curves.
    /// </summary>
    /// <remarks>
    /// This layer exists because the S3 core engine operates on sampled nodes and scalar fields rather than Rhino Breps directly.
    /// It performs meshing, node sampling, field visualization, planar base-layer seeding, and iso-curve extraction.
    /// </remarks>
    internal static class RhinoS3SlicerInterop
    {
        /// <summary>
        /// Slices a Brep using the S3-inspired field workflow and returns curves plus field diagnostics.
        /// </summary>
        /// <param name="brep">The source Brep. It should be valid and expressed in the active Rhino model units.</param>
        /// <param name="options">The S3 solve parameters. The value should not be <see langword="null"/>.</param>
        /// <param name="tolerance">The geometric tolerance used for meshing, intersection, joining, and projection operations. It should be greater than zero.</param>
        /// <returns>
        /// A <see cref="RhinoS3SliceResult"/> containing slice curves, planes, a diagnostic mesh, vector glyphs, and summary text.
        /// If the Brep cannot be meshed, the result contains no curves and an explanatory message.
        /// </returns>
        /// <remarks>
        /// Preconditions: callers should supply a valid Brep and positive tolerance. For meaningful output, the layer height must also be positive.
        /// Postconditions: the first layer is seeded from a planar XY intersection and higher layers are extracted from the S3-inspired scalar field.
        /// Exceptions: unexpected RhinoCommon failures may still bubble up from meshing, intersection, or curve operations.
        /// Differences: unlike <see cref="RhinoBrepLayerSlicer.Slice"/>, this workflow first solves an intermediate print-direction field and then extracts layers from that field.
        /// Side-effects: allocates Rhino meshes, curves, lines, and planes for the current solve only; does not modify document geometry.
        /// </remarks>
        public static RhinoS3SliceResult Slice(Brep brep, S3SliceGenerationOptions options, double tolerance)
        {
            var curves = new List<Curve>();
            var planes = new List<Plane>();

            Mesh analysisMesh = CreateAnalysisMesh(brep, options.LayerHeight, tolerance);
            if (analysisMesh.Vertices.Count == 0 || analysisMesh.Faces.Count == 0)
            {
                return new RhinoS3SliceResult(curves, planes, "Unable to create an analysis mesh from the Brep.", new Mesh(), Array.Empty<Line>(), "No histogram available.");
            }

            List<int>[] adjacency = BuildAdjacency(analysisMesh);
            Point3d[] vertices = Enumerable.Range(0, analysisMesh.Vertices.Count)
                .Select(index => analysisMesh.Vertices.Point3dAt(index))
                .ToArray();
            Vector3d[] normals = BuildVertexNormals(analysisMesh, adjacency);
            Vector3d[] supportDirections = normals.Select(normal => ProjectToTangentPlane(Vector3d.ZAxis, normal)).ToArray();
            Vector3d[] curvatureDirections = Enumerable.Range(0, analysisMesh.Vertices.Count)
                .Select(index => EstimateCurvatureTangent(vertices, normals, adjacency, index))
                .ToArray();

            var nodes = new List<S3NodeSample>(vertices.Length);
            for (int i = 0; i < vertices.Length; i++)
            {
                nodes.Add(new S3NodeSample(
                    i,
                    new Point3D(vertices[i].X, vertices[i].Y, vertices[i].Z),
                    ToCore(normals[i]),
                    ToCore(supportDirections[i]),
                    ToCore(curvatureDirections[i]),
                    adjacency[i]));
            }

            var engine = new S3SlicerEngine();
            S3SliceFieldResult field = engine.Generate(nodes, options);

            double baseZ = vertices.Min(vertex => vertex.Z);
            AddPlanarBaseLayer(curves, planes, brep, analysisMesh, options, tolerance, baseZ);

            double minLevel = 1.0;
            double maxLevel = Math.Floor(field.ScalarValues.DefaultIfEmpty(0.0).Max());

            for (double level = minLevel; level <= maxLevel; level += 1.0)
            {
                Curve[] levelCurves = ExtractIsoCurves(analysisMesh, field.ScalarValues, level, tolerance);
                foreach (Curve curve in levelCurves)
                {
                    if (curve == null || !curve.IsValid || curve.GetLength() <= tolerance)
                    {
                        continue;
                    }

                    Curve outputCurve = SmoothCurveOnMesh(curve, analysisMesh, options, tolerance);
                    outputCurve = ConvertCurveToOutputPolyline(outputCurve, brep, analysisMesh, options, tolerance);
                    curves.Add(outputCurve);
                    planes.Add(BuildRepresentativePlane(outputCurve, analysisMesh, field.OptimizedDirections.Select(ToRhino).ToArray(), tolerance));
                }
            }

            Mesh previewMesh = CreateAnglePreviewMesh(analysisMesh, field.DirectionAngles);
            List<Line> fieldLines = CreateFieldLines(analysisMesh, field.OptimizedDirections.Select(ToRhino).ToArray(), options.LayerHeight);
            string histogram = BuildAngleHistogram(field.DirectionAngles);
            string analysis = BuildAnalysis(curves, field);
            return new RhinoS3SliceResult(curves, planes, analysis, previewMesh, fieldLines, histogram);
        }

        private static void AddPlanarBaseLayer(
            List<Curve> curves,
            List<Plane> planes,
            Brep brep,
            Mesh analysisMesh,
            S3SliceGenerationOptions options,
            double tolerance,
            double baseZ)
        {
            Plane basePlane = new Plane(new Point3d(0.0, 0.0, baseZ), Vector3d.XAxis, Vector3d.YAxis);
            Curve[] planarCurves;
            Point3d[] intersectionPoints;
            if (!Intersection.BrepPlane(brep, basePlane, tolerance, out planarCurves, out intersectionPoints) || planarCurves == null)
            {
                return;
            }

            foreach (Curve curve in planarCurves)
            {
                if (curve == null || !curve.IsValid || curve.GetLength() <= tolerance)
                {
                    continue;
                }

                Curve outputCurve = SmoothCurveOnMesh(curve, analysisMesh, options, tolerance);
                outputCurve = ConvertCurveToOutputPolyline(outputCurve, brep, analysisMesh, options, tolerance);
                curves.Add(outputCurve);
                planes.Add(basePlane);
            }
        }

        private static Mesh CreateAnalysisMesh(Brep brep, double layerHeight, double tolerance)
        {
            var meshingParameters = new MeshingParameters
            {
                JaggedSeams = false,
                RefineGrid = true,
                SimplePlanes = false,
                MinimumEdgeLength = Math.Max(tolerance, layerHeight * 0.2),
                MaximumEdgeLength = Math.Max(layerHeight * 0.75, tolerance * 6.0),
                GridAmplification = 1.2,
                GridAspectRatio = 4.0
            };

            Mesh[] brepMeshes = Mesh.CreateFromBrep(brep, meshingParameters);
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

        private static List<int>[] BuildAdjacency(Mesh mesh)
        {
            var adjacency = Enumerable.Range(0, mesh.Vertices.Count)
                .Select(_ => new List<int>())
                .ToArray();

            for (int faceIndex = 0; faceIndex < mesh.Faces.Count; faceIndex++)
            {
                MeshFace face = mesh.Faces[faceIndex];
                AddEdge(adjacency, face.A, face.B);
                AddEdge(adjacency, face.B, face.C);
                AddEdge(adjacency, face.C, face.A);
                if (!face.IsTriangle)
                {
                    AddEdge(adjacency, face.C, face.D);
                    AddEdge(adjacency, face.D, face.A);
                }
            }

            return adjacency;
        }

        private static void AddEdge(List<int>[] adjacency, int a, int b)
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

        private static Vector3d[] BuildVertexNormals(Mesh mesh, List<int>[] adjacency)
        {
            var normals = new Vector3d[mesh.Vertices.Count];
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                Vector3f source = mesh.Normals[i];
                normals[i] = new Vector3d(source.X, source.Y, source.Z);
                if (!normals[i].Unitize())
                {
                    normals[i] = Vector3d.ZAxis;
                }

                if (normals[i] * Vector3d.ZAxis < 0.0)
                {
                    normals[i].Reverse();
                }
            }

            for (int iteration = 0; iteration < 2; iteration++)
            {
                var next = new Vector3d[normals.Length];
                for (int i = 0; i < normals.Length; i++)
                {
                    Vector3d averaged = normals[i];
                    foreach (int neighborIndex in adjacency[i])
                    {
                        averaged += normals[neighborIndex];
                    }

                    if (!averaged.Unitize())
                    {
                        averaged = normals[i];
                    }

                    next[i] = averaged;
                }

                normals = next;
            }

            return normals;
        }

        private static Vector3d EstimateCurvatureTangent(Point3d[] vertices, Vector3d[] normals, List<int>[] adjacency, int vertexIndex)
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

                Vector3d tangent = ProjectToTangentPlane(edge, normal);
                if (!tangent.Unitize())
                {
                    continue;
                }

                double score = (normals[neighborIndex] - normals[vertexIndex]).Length / edgeLength;
                if (score > bestScore)
                {
                    bestScore = score;
                    bestDirection = tangent;
                }
            }

            if (!bestDirection.Unitize())
            {
                bestDirection = ProjectToTangentPlane(Vector3d.XAxis, normal);
            }

            if (!bestDirection.Unitize())
            {
                bestDirection = Vector3d.ZAxis;
            }

            if (bestDirection * Vector3d.ZAxis < 0.0)
            {
                bestDirection.Reverse();
            }

            return bestDirection;
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

        private static Curve[] ExtractIsoCurves(Mesh mesh, IReadOnlyList<double> scalarValues, double level, double tolerance)
        {
            var segments = new List<LineCurve>();
            for (int faceIndex = 0; faceIndex < mesh.Faces.Count; faceIndex++)
            {
                MeshFace face = mesh.Faces[faceIndex];
                if (face.IsTriangle)
                {
                    AddTriangleIsoSegment(mesh, scalarValues, level, tolerance, face.A, face.B, face.C, segments);
                }
                else
                {
                    AddTriangleIsoSegment(mesh, scalarValues, level, tolerance, face.A, face.B, face.C, segments);
                    AddTriangleIsoSegment(mesh, scalarValues, level, tolerance, face.A, face.C, face.D, segments);
                }
            }

            if (segments.Count == 0)
            {
                return Array.Empty<Curve>();
            }

            Curve[] joined = Curve.JoinCurves(segments.Cast<Curve>().ToArray(), tolerance * 4.0);
            return joined.Length > 0 ? joined : segments.Cast<Curve>().ToArray();
        }

        private static void AddTriangleIsoSegment(Mesh mesh, IReadOnlyList<double> scalarValues, double level, double tolerance, int a, int b, int c, List<LineCurve> segments)
        {
            var points = new List<Point3d>(3);
            TryAddIsoPoint(mesh, scalarValues, level, a, b, points);
            TryAddIsoPoint(mesh, scalarValues, level, b, c, points);
            TryAddIsoPoint(mesh, scalarValues, level, c, a, points);
            DeduplicatePoints(points, tolerance * 2.0);

            if (points.Count == 2 && points[0].DistanceToSquared(points[1]) > tolerance * tolerance)
            {
                segments.Add(new LineCurve(points[0], points[1]));
            }
        }

        private static void TryAddIsoPoint(Mesh mesh, IReadOnlyList<double> scalarValues, double level, int startIndex, int endIndex, List<Point3d> points)
        {
            double startValue = scalarValues[startIndex];
            double endValue = scalarValues[endIndex];
            double epsilon = 1e-8;
            if ((startValue < level - epsilon && endValue < level - epsilon) || (startValue > level + epsilon && endValue > level + epsilon))
            {
                return;
            }

            Point3d start = mesh.Vertices.Point3dAt(startIndex);
            Point3d end = mesh.Vertices.Point3dAt(endIndex);
            if (Math.Abs(startValue - endValue) <= epsilon)
            {
                points.Add(start);
                points.Add(end);
                return;
            }

            double t = (level - startValue) / (endValue - startValue);
            t = Math.Max(0.0, Math.Min(1.0, t));
            points.Add(start + ((end - start) * t));
        }

        private static Curve SmoothCurveOnMesh(Curve curve, Mesh mesh, S3SliceGenerationOptions options, double tolerance)
        {
            double smoothingFactor = Math.Max(0.0, Math.Min(1.0, options.OutputCurveSmoothingFactor));
            int iterations = Math.Max(0, options.OutputCurveSmoothingIterations);
            double allowedDeviation = Math.Max(tolerance, options.SurfaceFollowTolerance);
            if (smoothingFactor <= 0.0 || iterations == 0)
            {
                return curve;
            }

            int divisionCount = Math.Max(12, (int)Math.Ceiling(curve.GetLength() / Math.Max(tolerance * 8.0, options.LayerHeight * 0.35)));
            double[] parameters = curve.DivideByCount(divisionCount, true);
            if (parameters == null || parameters.Length < 4)
            {
                return curve;
            }

            bool closed = curve.IsClosed;
            var original = parameters.Select(curve.PointAt).ToList();
            var points = new List<Point3d>(original);

            for (int iteration = 0; iteration < iterations; iteration++)
            {
                var next = new List<Point3d>(points);
                for (int i = 0; i < points.Count; i++)
                {
                    if (!closed && (i == 0 || i == points.Count - 1))
                    {
                        next[i] = original[i];
                        continue;
                    }

                    int prev = i == 0 ? points.Count - 1 : i - 1;
                    int nxt = i == points.Count - 1 ? 0 : i + 1;
                    Point3d candidate = LerpPoint(points[i], Midpoint(points[prev], points[nxt]), smoothingFactor);
                    candidate = ProjectPointToMesh(mesh, candidate, allowedDeviation * 4.0, points[i]);
                    next[i] = ClampPointDeviation(original[i], candidate, allowedDeviation);
                }

                points = next;
            }

            Curve smoothCurve = Curve.CreateInterpolatedCurve(points, 3);
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

        private static Curve ConvertCurveToOutputPolyline(Curve curve, Brep brep, Mesh mesh, S3SliceGenerationOptions options, double tolerance)
        {
            bool closed = curve.IsClosed;
            int minimumPointCount = closed ? 3 : 2;
            if (options.OutputSamplingMode == OutputPolylineSamplingMode.None)
            {
                return curve;
            }

            List<double> sampleLocations = options.OutputSamplingMode == OutputPolylineSamplingMode.PointCount
                ? BuildFeatureAwareLocationsByCount(curve, options.OutputPolylinePointCount, tolerance)
                : BuildFeatureAwareLocationsByDistance(curve, options.OutputPolylineMinPointDistance, tolerance);

            if (sampleLocations.Count < minimumPointCount)
            {
                return curve;
            }

            var points = sampleLocations
                .Select(location => ProjectCurveSample(curve, location, brep, mesh, options.SurfaceFollowTolerance, tolerance))
                .ToList();

            if (closed && points.Count > 0)
            {
                points.Add(points[0]);
            }

            DeduplicateSequentialPoints(points, tolerance);
            Polyline polyline = new(points);
            return !polyline.IsValid || polyline.Count < minimumPointCount ? curve : new PolylineCurve(polyline);
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

            double baseSpacing = closed ? 1.0 / targetCount : 1.0 / Math.Max(1, targetCount - 1);
            List<FeatureLocation> anchors = CollectFeatureLocations(curve, curve.GetLength() * baseSpacing, tolerance);
            var selected = new List<double>();

            foreach (FeatureLocation anchor in anchors.OrderByDescending(location => location.Score))
            {
                if (selected.Count >= targetCount)
                {
                    break;
                }

                if (ContainsLocationNear(selected, anchor.NormalizedLength, baseSpacing * 0.5, closed))
                {
                    continue;
                }

                selected.Add(anchor.NormalizedLength);
            }

            while (selected.Count < targetCount)
            {
                double nextLocation = FindLargestGapMidpoint(selected, closed);
                if (ContainsLocationNear(selected, nextLocation, baseSpacing * 0.1, closed))
                {
                    break;
                }

                selected.Add(nextLocation);
            }

            return selected.OrderBy(value => value).ToList();
        }

        private static List<double> BuildFeatureAwareLocationsByDistance(Curve curve, double minimumPointDistance, double tolerance)
        {
            bool closed = curve.IsClosed;
            int minimumPointCount = closed ? 3 : 2;
            double curveLength = curve.GetLength();
            if (curveLength <= tolerance || minimumPointDistance <= tolerance)
            {
                return new List<double>();
            }

            double normalizedSpacing = Math.Min(1.0, Math.Max(tolerance / curveLength, minimumPointDistance / curveLength));
            List<FeatureLocation> anchors = CollectFeatureLocations(curve, minimumPointDistance, tolerance);
            var selected = new List<double>();

            foreach (FeatureLocation anchor in anchors.OrderByDescending(location => location.Score))
            {
                if (ContainsLocationNear(selected, anchor.NormalizedLength, normalizedSpacing, closed))
                {
                    continue;
                }

                selected.Add(anchor.NormalizedLength);
            }

            if (selected.Count < minimumPointCount)
            {
                if (!closed)
                {
                    if (!ContainsLocationNear(selected, 0.0, normalizedSpacing * 0.25, false))
                    {
                        selected.Add(0.0);
                    }

                    if (!ContainsLocationNear(selected, 1.0, normalizedSpacing * 0.25, false))
                    {
                        selected.Add(1.0);
                    }
                }
                else if (selected.Count == 0)
                {
                    selected.Add(0.0);
                }
            }

            return FillGapsWithMinimumSpacing(selected, normalizedSpacing, closed);
        }

        private static List<FeatureLocation> CollectFeatureLocations(Curve curve, double spacingHint, double tolerance)
        {
            bool closed = curve.IsClosed;
            var locations = new List<FeatureLocation>();
            if (closed)
            {
                locations.Add(new FeatureLocation(0.0, double.MaxValue));
            }
            else
            {
                locations.Add(new FeatureLocation(0.0, double.MaxValue));
                locations.Add(new FeatureLocation(1.0, double.MaxValue));
            }

            locations.AddRange(CollectPolylineFeatures(curve, tolerance));
            locations.AddRange(CollectDenseFeatures(curve, spacingHint, tolerance));
            return locations;
        }

        private static IEnumerable<FeatureLocation> CollectPolylineFeatures(Curve curve, double tolerance)
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

        private static IEnumerable<FeatureLocation> CollectDenseFeatures(Curve curve, double spacingHint, double tolerance)
        {
            var features = new List<FeatureLocation>();
            bool closed = curve.IsClosed;
            double curveLength = curve.GetLength();
            int denseCount = Math.Max((int)Math.Ceiling(curveLength / Math.Max(spacingHint * 0.35, tolerance * 6.0)), closed ? 48 : 32);
            List<double> parameters = SampleCurveParameters(curve, denseCount, closed);
            List<Point3d> points = parameters.Select(curve.PointAt).ToList();

            for (int i = 0; i < points.Count; i++)
            {
                bool endpoint = !closed && (i == 0 || i == points.Count - 1);
                if (endpoint)
                {
                    continue;
                }

                int prev = i == 0 ? points.Count - 1 : i - 1;
                int next = i == points.Count - 1 ? 0 : i + 1;
                double turnAngle = ComputeTurnAngle(points[prev], points[i], points[next]);
                if (turnAngle < 10.0)
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
                    parameters.Add(curve.Domain.ParameterAt((double)i / count));
                }
            }
            else
            {
                for (int i = 0; i < count; i++)
                {
                    parameters.Add(curve.Domain.ParameterAt(count == 1 ? 0.0 : (double)i / (count - 1)));
                }
            }

            return parameters;
        }

        private static bool ContainsLocationNear(IReadOnlyList<double> locations, double candidate, double spacing, bool closed)
        {
            foreach (double location in locations)
            {
                double distance = Math.Abs(location - candidate);
                if (distance <= spacing)
                {
                    return true;
                }

                if (closed && Math.Min(distance, 1.0 - distance) <= spacing)
                {
                    return true;
                }
            }

            return false;
        }

        private static List<double> FillGapsWithMinimumSpacing(IReadOnlyList<double> selectedLocations, double spacing, bool closed)
        {
            if (selectedLocations.Count == 0)
            {
                return new List<double>();
            }

            List<double> sorted = selectedLocations.OrderBy(value => value).ToList();
            var filled = new List<double>();

            if (!closed)
            {
                for (int i = 0; i < sorted.Count - 1; i++)
                {
                    if (i == 0)
                    {
                        filled.Add(sorted[i]);
                    }

                    AppendIntermediateLocations(filled, sorted[i], sorted[i + 1], spacing);
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

                    AppendIntermediateLocations(filled, start, end, spacing);
                }

                for (int i = 0; i < filled.Count; i++)
                {
                    if (filled[i] >= 1.0)
                    {
                        filled[i] -= 1.0;
                    }
                }
            }

            return filled.Distinct().OrderBy(value => value).ToList();
        }

        private static void AppendIntermediateLocations(List<double> filled, double start, double end, double spacing)
        {
            double gap = end - start;
            int segments = Math.Max(1, (int)Math.Floor(gap / Math.Max(spacing, 1e-6)));
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

        private static double FindLargestGapMidpoint(IReadOnlyList<double> locations, bool closed)
        {
            if (locations.Count == 0)
            {
                return 0.5;
            }

            List<double> sorted = locations.OrderBy(value => value).ToList();
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

            return midpoint;
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

            double partialLength = curve.GetLength(new Interval(curve.Domain.T0, parameter));
            return Math.Max(0.0, Math.Min(1.0, partialLength / totalLength));
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

        private static Point3d ProjectCurveSample(Curve curve, double normalizedLength, Brep brep, Mesh mesh, double surfaceFollowTolerance, double tolerance)
        {
            double parameter = curve.Domain.ParameterAt(normalizedLength);
            if (!curve.NormalizedLengthParameter(normalizedLength, out parameter))
            {
                parameter = curve.Domain.ParameterAt(normalizedLength);
            }

            Point3d point = curve.PointAt(parameter);
            Point3d projectedToBrep = ProjectPointToBrep(brep, point, tolerance);
            if (projectedToBrep.DistanceTo(point) <= Math.Max(surfaceFollowTolerance, tolerance))
            {
                return projectedToBrep;
            }

            Point3d projectedToMesh = ProjectPointToMesh(mesh, point, Math.Max(surfaceFollowTolerance, tolerance) * 4.0, point);
            return ClampPointDeviation(point, ProjectPointToBrep(brep, projectedToMesh, tolerance), Math.Max(surfaceFollowTolerance, tolerance));
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

        private static Vector3d AverageFieldVectorAlongCurve(Curve curve, Mesh mesh, IReadOnlyList<Vector3d> fieldVectors, double tolerance)
        {
            Vector3d average = Vector3d.Zero;
            double[] parameters = curve.DivideByCount(12, true);
            if (parameters == null || parameters.Length == 0)
            {
                parameters = new[] { curve.Domain.ParameterAt(0.5) };
            }

            foreach (double parameter in parameters)
            {
                MeshPoint meshPoint = mesh.ClosestMeshPoint(curve.PointAt(parameter), Math.Max(tolerance * 10.0, 1.0));
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

            return average;
        }

        private static Mesh CreateAnglePreviewMesh(Mesh sourceMesh, IReadOnlyList<double> angles)
        {
            Mesh preview = sourceMesh.DuplicateMesh();
            preview.VertexColors.Clear();
            double maxAngle = Math.Max(1.0, angles.DefaultIfEmpty(0.0).Max());
            for (int i = 0; i < preview.Vertices.Count; i++)
            {
                double angle = i < angles.Count ? angles[i] : 0.0;
                preview.VertexColors.Add(MapAngleToColor(angle, maxAngle));
            }

            return preview;
        }

        private static List<Line> CreateFieldLines(Mesh mesh, IReadOnlyList<Vector3d> directions, double layerHeight)
        {
            var lines = new List<Line>();
            int step = Math.Max(1, mesh.Vertices.Count / 300);
            double glyphLength = Math.Max(layerHeight * 0.6, 1.0);

            for (int i = 0; i < mesh.Vertices.Count; i += step)
            {
                Vector3d direction = i < directions.Count ? directions[i] : Vector3d.ZAxis;
                if (!direction.Unitize())
                {
                    continue;
                }

                Point3d origin = mesh.Vertices.Point3dAt(i);
                lines.Add(new Line(origin, origin + (direction * glyphLength)));
            }

            return lines;
        }

        private static string BuildAngleHistogram(IReadOnlyList<double> angles)
        {
            if (angles.Count == 0)
            {
                return "No angle samples available.";
            }

            double histogramMax = Math.Max(5.0, angles.Max());
            int binCount = 12;
            double binSize = histogramMax / binCount;
            int[] counts = new int[binCount];

            foreach (double angle in angles)
            {
                int index = Math.Min(binCount - 1, (int)Math.Floor(angle / binSize));
                counts[index]++;
            }

            var lines = new List<string>
            {
                $"Angle Histogram -> samples: {angles.Count}, range: 0.0 to {histogramMax:F1} deg"
            };

            for (int i = 0; i < binCount; i++)
            {
                double start = i * binSize;
                double end = (i + 1) * binSize;
                double percent = 100.0 * counts[i] / Math.Max(1, angles.Count);
                lines.Add($"{start,6:F1} - {end,6:F1} deg : {counts[i],5} ({percent,5:F1}%)");
            }

            return string.Join(Environment.NewLine, lines);
        }

        private static string BuildAnalysis(IReadOnlyList<Curve> curves, S3SliceFieldResult field)
        {
            List<double> segmentLengths = ExtractSegmentLengths(curves);
            double minDeformation = field.DeformationScales.DefaultIfEmpty(0.0).Min();
            double maxDeformation = field.DeformationScales.DefaultIfEmpty(0.0).Max();
            double avgAngle = field.DirectionAngles.DefaultIfEmpty(0.0).Average();

            return
                $"S3 Approximation -> curves: {curves.Count}, field angle avg: {avgAngle:F3}, deformation range: {minDeformation:F3} to {maxDeformation:F3}{Environment.NewLine}" +
                $"Segment Lengths -> min: {segmentLengths.DefaultIfEmpty(0.0).Min():F3}, max: {segmentLengths.DefaultIfEmpty(0.0).Max():F3}, avg: {segmentLengths.DefaultIfEmpty(0.0).Average():F3}";
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
                }
            }

            return lengths;
        }

        private static Point3d ProjectPointToMesh(Mesh mesh, Point3d point, double searchDistance, Point3d fallback)
        {
            MeshPoint meshPoint = mesh.ClosestMeshPoint(point, Math.Max(searchDistance, 1.0));
            return meshPoint == null ? fallback : meshPoint.Point;
        }

        private static Point3d ProjectPointToBrep(Brep brep, Point3d point, double tolerance)
        {
            Point3d closestPoint;
            ComponentIndex componentIndex;
            double s;
            double t;
            Vector3d normal;
            return brep.ClosestPoint(point, out closestPoint, out componentIndex, out s, out t, tolerance, out normal)
                ? closestPoint
                : point;
        }

        private static Point3d ClampPointDeviation(Point3d original, Point3d candidate, double maxDeviation)
        {
            Vector3d deviation = candidate - original;
            double length = deviation.Length;
            if (length <= maxDeviation || length <= 1e-9)
            {
                return candidate;
            }

            deviation.Unitize();
            return original + (deviation * maxDeviation);
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

        private static Point3d Midpoint(Point3d a, Point3d b) => new((a.X + b.X) * 0.5, (a.Y + b.Y) * 0.5, (a.Z + b.Z) * 0.5);

        private static Point3d LerpPoint(Point3d a, Point3d b, double t)
        {
            double clamped = Math.Max(0.0, Math.Min(1.0, t));
            return new Point3d(
                a.X + ((b.X - a.X) * clamped),
                a.Y + ((b.Y - a.Y) * clamped),
                a.Z + ((b.Z - a.Z) * clamped));
        }

        private static Color MapAngleToColor(double angle, double maxAngle)
        {
            double t = Math.Max(0.0, Math.Min(1.0, angle / Math.Max(1.0, maxAngle)));
            if (t <= 0.5)
            {
                return InterpolateColor(Color.FromArgb(60, 120, 255), Color.FromArgb(255, 220, 80), t * 2.0);
            }

            return InterpolateColor(Color.FromArgb(255, 220, 80), Color.FromArgb(220, 70, 40), (t - 0.5) * 2.0);
        }

        private static Color InterpolateColor(Color start, Color end, double t)
        {
            double clamped = Math.Max(0.0, Math.Min(1.0, t));
            int r = (int)Math.Round(start.R + ((end.R - start.R) * clamped));
            int g = (int)Math.Round(start.G + ((end.G - start.G) * clamped));
            int b = (int)Math.Round(start.B + ((end.B - start.B) * clamped));
            return Color.FromArgb(r, g, b);
        }

        private static Vector3D ToCore(Vector3d vector) => new(vector.X, vector.Y, vector.Z);

        private static Vector3d ToRhino(Vector3D vector) => new(vector.X, vector.Y, vector.Z);

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
}
