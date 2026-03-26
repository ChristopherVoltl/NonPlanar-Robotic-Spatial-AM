using System;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using SpatialAdditiveManufacturing.Core.Slicing;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Grasshopper component for an S3-style slicing workflow inspired by the
    /// SIGGRAPH Asia 2022 paper and public reference implementation.
    /// This is an approximation built around an inner orientation-field loop
    /// and an outer deformation/scalar-field loop.
    /// </summary>
    /// <remarks>
    /// This component exists to expose the S3-inspired solver separately from the topology-driven slicer so both workflows
    /// can coexist in the same plugin. It is intentionally field-focused and does not talk to SMT directly.
    /// </remarks>
    public class S3SliceComponent : GH_Component
    {
        /// <summary>
        /// Initializes the Grasshopper component metadata used for registration and UI display.
        /// </summary>
        /// <remarks>
        /// Preconditions: none.
        /// Postconditions: the component is registered in the FGAM/SpatialPrinting tab when the assembly loads.
        /// Exceptions: none.
        /// Side-effects: none beyond standard Grasshopper component registration.
        /// </remarks>
        public S3SliceComponent()
            : base("S3 Slice", "S3 Slice", "Approximate S3-style curved slicing with quaternion-like field optimization and deformation-driven scalar extraction.", "FGAM", "SpatialPrinting")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "Input Brep.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Layer Height", "LH", "Target scalar layer height.", GH_ParamAccess.item, 5.0);
            pManager.AddIntegerParameter("Inner Iterations", "II", "Iterations of the quaternion-like orientation smoothing loop.", GH_ParamAccess.item, 20);
            pManager.AddIntegerParameter("Outer Iterations", "OI", "Iterations of the deformation/scalar update loop.", GH_ParamAccess.item, 14);
            pManager.AddNumberParameter("Field Smoothness", "FS", "Neighbor influence in the inner orientation-field loop. Lower values preserve more local variation.", GH_ParamAccess.item, 0.3);
            pManager.AddNumberParameter("Deformation Strength", "DS", "Strength of the outer deformation update and field-following scalar bend.", GH_ParamAccess.item, 0.75);
            pManager.AddNumberParameter("Support Weight", "SW", "Weight for the support-free objective proxy.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Strength Weight", "STW", "Weight for the strength objective proxy.", GH_ParamAccess.item, 0.75);
            pManager.AddNumberParameter("Quality Weight", "QW", "Weight for the surface quality objective proxy.", GH_ParamAccess.item, 0.75);
            pManager.AddNumberParameter("Curve Smoothing", "CS", "Optional post-smoothing for output curves.", GH_ParamAccess.item, 0.0);
            pManager.AddIntegerParameter("Sampling Mode", "SM", "0 = no resampling, 1 = point count, 2 = minimum distance.", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("Point Count", "PC", "If Sampling Mode = 1, resample using this many points.", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Min Point Dist", "MPD", "If Sampling Mode = 2, minimum distance between output points.", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Surface Tol", "ST", "Maximum deviation allowed when fitting samples back to the Brep.", GH_ParamAccess.item, 1.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Sliced Curves", "C", "Generated S3-style slice curves.", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Slice Planes", "P", "Representative planes along the generated curves.", GH_ParamAccess.list);
            pManager.AddTextParameter("Analysis", "A", "Text summary of the S3-style field and output geometry.", GH_ParamAccess.item);
            pManager.AddMeshParameter("Field Map", "M", "Mesh preview colored by field angle.", GH_ParamAccess.item);
            pManager.AddLineParameter("Field Vectors", "V", "Diagnostic field vectors sampled across the Brep surface.", GH_ParamAccess.list);
            pManager.AddTextParameter("Field Histogram", "H", "Histogram of field angles on the Brep surface.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Reads the Grasshopper inputs, solves the S3-inspired field, extracts iso-curves, and publishes diagnostics.
        /// </summary>
        /// <param name="DA">Grasshopper's data-access wrapper for retrieving inputs and assigning outputs.</param>
        /// <remarks>
        /// Preconditions: callers must provide a valid Brep and should use positive layer heights and sensible iteration counts.
        /// The weight inputs are relative objective weights rather than normalized percentages.
        /// Postconditions: outputs include slice curves, representative planes, a field map, vector glyphs, and analysis text.
        /// Exceptions: unexpected Rhino or solver failures may still bubble up from the interop or core engine layers.
        /// Differences: unlike <see cref="NonPlanarSliceComponent"/>, this component solves an intermediate print-direction field first
        /// and then extracts curves from that field.
        /// Side-effects: emits Grasshopper runtime warnings or errors and allocates Rhino output geometry.
        /// </remarks>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep? brep = null;
            double layerHeight = 5.0;
            int innerIterations = 20;
            int outerIterations = 14;
            double fieldSmoothness = 0.3;
            double deformationStrength = 0.75;
            double supportWeight = 1.0;
            double strengthWeight = 0.75;
            double qualityWeight = 0.75;
            double curveSmoothing = 0.0;
            int samplingMode = 0;
            int pointCount = 0;
            double minPointDistance = 0.0;
            double surfaceTolerance = 1.0;

            if (!DA.GetData(0, ref brep)) return;
            if (!DA.GetData(1, ref layerHeight)) return;
            if (!DA.GetData(2, ref innerIterations)) return;
            if (!DA.GetData(3, ref outerIterations)) return;
            if (!DA.GetData(4, ref fieldSmoothness)) return;
            if (!DA.GetData(5, ref deformationStrength)) return;
            if (!DA.GetData(6, ref supportWeight)) return;
            if (!DA.GetData(7, ref strengthWeight)) return;
            if (!DA.GetData(8, ref qualityWeight)) return;
            if (!DA.GetData(9, ref curveSmoothing)) return;
            if (!DA.GetData(10, ref samplingMode)) return;
            if (!DA.GetData(11, ref pointCount)) return;
            if (!DA.GetData(12, ref minPointDistance)) return;
            if (!DA.GetData(13, ref surfaceTolerance)) return;

            if (brep == null || !brep.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "A valid Brep is required.");
                return;
            }

            double tolerance = RhinoDoc.ActiveDoc?.ModelAbsoluteTolerance ?? 0.01;
            var result = RhinoS3SlicerInterop.Slice(
                brep,
                new S3SliceGenerationOptions
                {
                    LayerHeight = layerHeight,
                    InnerIterations = innerIterations,
                    OuterIterations = outerIterations,
                    QuaternionSmoothness = fieldSmoothness,
                    DeformationStrength = deformationStrength,
                    SupportFreeWeight = supportWeight,
                    StrengthWeight = strengthWeight,
                    SurfaceQualityWeight = qualityWeight,
                    OutputCurveSmoothingFactor = curveSmoothing,
                    OutputCurveSmoothingIterations = curveSmoothing <= 0.0 ? 0 : 5,
                    OutputSamplingMode = samplingMode <= 0 ? OutputPolylineSamplingMode.None : (samplingMode == 1 ? OutputPolylineSamplingMode.PointCount : OutputPolylineSamplingMode.MinimumDistance),
                    OutputPolylinePointCount = pointCount,
                    OutputPolylineMinPointDistance = minPointDistance,
                    SurfaceFollowTolerance = surfaceTolerance,
                },
                tolerance);

            DA.SetDataList(0, result.Curves);
            DA.SetDataList(1, result.Planes);
            DA.SetData(2, result.Analysis);
            DA.SetData(3, result.FieldPreviewMesh);
            DA.SetDataList(4, result.FieldLines);
            DA.SetData(5, result.FieldHistogram);

            if (result.Curves.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No S3-style slice curves were produced for the current settings.");
            }
        }

        protected override System.Drawing.Bitmap Icon => null!;

        public override Guid ComponentGuid => new Guid("15f08bb5-2dde-4581-8f64-d20ce0ef79f9");
    }
}
