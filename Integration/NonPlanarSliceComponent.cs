using System;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using SpatialAdditiveManufacturing.Core.Slicing;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Grasshopper component responsible only for:
    /// Brep -> topology sections -> nonplanar slices -> Rhino output geometry.
    ///
    /// The slicer implementation is inspired by the referenced paper's workflow:
    /// critical-feature detection, geometry-conforming scalar field construction,
    /// saddle bias, and adaptive slice extraction.
    /// </summary>
    public class NonPlanarSliceComponent : GH_Component
    {
        public NonPlanarSliceComponent()
            : base("NonPlanar Slice", "NP Slice", "Slice a Brep into topology-driven nonplanar curves.", "FGAM", "SpatialPrinting")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "Input Brep to deconstruct into topology-driven sections.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Field Mode", "FM", "Surface field driver. 0 = surface normal, 1 = surface tangent.", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("Angle Influence", "AI", "Percentage controlling how strongly the scalar field departs from planar XY layering toward the selected Brep surface direction.", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Smoothing", "S", "Blend factor between neighboring slice normals and field values.", GH_ParamAccess.item, 0.35);
            pManager.AddNumberParameter("Layer Height", "LH", "Distance between successive base XY slicing layers.", GH_ParamAccess.item, 5.0);
            pManager.AddNumberParameter("Adaptive Blend", "AB", "Blend between slope-driven and curvature-driven scalar-field influence.", GH_ParamAccess.item, 0.5);
            pManager.AddNumberParameter("Saddle Bias", "SB", "Strength of the signed-curvature bias in the scalar field.", GH_ParamAccess.item, 0.35);
            pManager.AddNumberParameter("Curve Smoothing", "CS", "Optional post-smoothing for slice curves before final output.", GH_ParamAccess.item, 0.0);
            pManager.AddIntegerParameter("Sampling Mode", "SM", "0 = no resampling, 1 = point count, 2 = minimum distance. Corner features are prioritized in both modes.", GH_ParamAccess.item, 2);
            pManager.AddIntegerParameter("Point Count", "PC", "If Sampling Mode = 1, resample each output as a polyline with this many points while prioritizing corners.", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Min Point Dist", "MPD", "If Sampling Mode = 2, minimum distance allowed between output polyline points. Sharp features are prioritized first.", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Surface Tol", "ST", "Maximum allowed deviation from the raw contour while smoothing.", GH_ParamAccess.item, 1.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Sliced Curves", "C", "Generated nonplanar slice curves.", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Slice Planes", "P", "Planes aligned to the generated slice frames.", GH_ParamAccess.list);
            pManager.AddTextParameter("Analysis", "A", "Text summary of segment length and angle statistics.", GH_ParamAccess.item);
            pManager.AddMeshParameter("Angle Map", "M", "Mesh preview colored by vector-field angle from the global Z axis.", GH_ParamAccess.item);
            pManager.AddLineParameter("Field Vectors", "V", "Diagnostic field vectors sampled across the Brep surface.", GH_ParamAccess.list);
            pManager.AddTextParameter("Angle Histogram", "H", "Histogram of vector-field angles over the Brep surface.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep? brep = null;
            int fieldMode = 1;
            double angleInfluence = 0.0;
            double smoothing = 0.35;
            double targetLayerHeight = 5.0;
            double adaptiveBlend = 0.5;
            double saddleBias = 0.35;
            double curveSmoothing = 0.0;
            int outputSamplingMode = 2;
            int outputPolylinePointCount = 0;
            double outputPolylineMinPointDistance = 0.0;
            double surfaceFollowTolerance = 1.0;

            if (!DA.GetData(0, ref brep)) return;
            if (!DA.GetData(1, ref fieldMode)) return;
            if (!DA.GetData(2, ref angleInfluence)) return;
            if (!DA.GetData(3, ref smoothing)) return;
            if (!DA.GetData(4, ref targetLayerHeight)) return;
            if (!DA.GetData(5, ref adaptiveBlend)) return;
            if (!DA.GetData(6, ref saddleBias)) return;
            if (!DA.GetData(7, ref curveSmoothing)) return;
            if (!DA.GetData(8, ref outputSamplingMode)) return;
            if (!DA.GetData(9, ref outputPolylinePointCount)) return;
            if (!DA.GetData(10, ref outputPolylineMinPointDistance)) return;
            if (!DA.GetData(11, ref surfaceFollowTolerance)) return;

            if (brep == null || !brep.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "A valid Brep is required.");
                return;
            }

            double tolerance = RhinoDoc.ActiveDoc?.ModelAbsoluteTolerance ?? 0.01;
            var result = RhinoBrepLayerSlicer.Slice(
                brep,
                new SliceGenerationOptions
                {
                    FieldMode = fieldMode <= 0 ? SurfaceFieldMode.SurfaceNormal : SurfaceFieldMode.SurfaceTangent,
                    AngleInfluencePercent = angleInfluence,
                    SmoothingFactor = smoothing,
                    TargetLayerHeight = targetLayerHeight,
                    AdaptiveBlendFactor = adaptiveBlend,
                    SaddleBiasStrength = saddleBias,
                    OutputCurveSmoothingFactor = curveSmoothing,
                    OutputCurveSmoothingIterations = curveSmoothing <= 0.0 ? 0 : 5,
                    OutputSamplingMode = outputSamplingMode <= 0 ? OutputPolylineSamplingMode.None : (outputSamplingMode == 1 ? OutputPolylineSamplingMode.PointCount : OutputPolylineSamplingMode.MinimumDistance),
                    OutputPolylinePointCount = outputPolylinePointCount,
                    OutputPolylineMinPointDistance = outputPolylineMinPointDistance,
                    SurfaceFollowTolerance = surfaceFollowTolerance,
                },
                tolerance);

            DA.SetDataList(0, result.Curves);
            DA.SetDataList(1, result.Planes);
            DA.SetData(2, result.Analysis);
            DA.SetData(3, result.AnglePreviewMesh);
            DA.SetDataList(4, result.FieldLines);
            DA.SetData(5, result.AngleHistogram);

            if (result.Curves.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No section curves were produced from the Brep for the current layer settings.");
            }
        }

        protected override System.Drawing.Bitmap Icon => null!;

        public override Guid ComponentGuid => new Guid("641b9cb3-5dbe-49f2-a329-9e6656d10f33");
    }
}
