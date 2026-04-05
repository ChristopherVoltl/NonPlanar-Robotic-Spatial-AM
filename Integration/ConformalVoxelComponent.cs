using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Grasshopper component that turns ordered nonplanar layer curves into a conformal interior voxel scaffold.
    /// </summary>
    /// <remarks>
    /// This component exists to bridge layer-based slicing into volumetric spatial-print structures. It differs from
    /// <see cref="SpatialLatticeComponent"/> by creating normalized voxel bands and cell centers instead of a truss-only
    /// member network. Side-effects are limited to Grasshopper runtime messages and output assignment.
    /// </remarks>
    public class ConformalVoxelComponent : GH_Component
    {
        /// <summary>
        /// Initializes the metadata used for Grasshopper registration and UI display.
        /// </summary>
        public ConformalVoxelComponent()
            : base("Conformal Voxels", "Voxels", "Generate a conformal interior voxel grid from ordered nonplanar layers.", "FGAM", "SpatialPrinting")
        {
        }

        /// <summary>
        /// Registers the input parameters for the conformal voxel generator.
        /// </summary>
        /// <remarks>
        /// Preconditions: layer curves should be closed and already ordered through the slice stack. Guide curves are optional.
        /// </remarks>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Layer Curves", "C", "Ordered closed layer curves. Input order defines stack order.", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Layer Planes", "P", "Optional representative planes matching the layer order.", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager.AddCurveParameter("Start Guide", "SG", "Optional guide curve used to anchor the startpoint of every normalized layer.", GH_ParamAccess.item);
            pManager[2].Optional = true;
            pManager.AddCurveParameter("Direction Guide", "DG", "Optional guide curve used to bias the forward direction after the seam is fixed.", GH_ParamAccess.item);
            pManager[3].Optional = true;
            pManager.AddNumberParameter("Min Voxel Size", "Min", "Preferred lower bound for voxel spacing in model units.", GH_ParamAccess.item, 5.0);
            pManager.AddNumberParameter("Max Voxel Size", "Max", "Preferred upper bound for voxel spacing in model units.", GH_ParamAccess.item, 15.0);
            pManager.AddNumberParameter("Fallback Span", "FS", "Fallback segment length used only when an input layer is not already a usable polyline.", GH_ParamAccess.item, 10.0);
        }

        /// <summary>
        /// Registers the outputs for the conformal voxel scaffold.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Ring Curves", "R", "Normalized perimeter and interior voxel rings in layer-major order.", GH_ParamAccess.list);
            pManager.AddLineParameter("Voxel Edges", "E", "Unique voxel edge lines for the current scaffold.", GH_ParamAccess.list);
            pManager.AddPointParameter("Voxel Centers", "VC", "Centers of the voxel cells between adjacent layers and ring bands.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Face Loops", "F", "Representative quad face loops for previewing the voxel cells.", GH_ParamAccess.list);
            pManager.AddPointParameter("Nodes", "N", "Unique node locations used by the voxel scaffold.", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Perimeter Divisions", "PD", "Shared perimeter division count used on every layer.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Radial Bands", "RB", "Shared inward voxel band count used on every layer.", GH_ParamAccess.item);
            pManager.AddTextParameter("Analysis", "A", "Diagnostic summary of the current conformal voxel solve.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Solves the current Grasshopper instance by building the voxel scaffold from the supplied layer stack.
        /// </summary>
        /// <remarks>
        /// Preconditions: at least one closed layer curve is required. Postconditions: outputs are populated even when
        /// generation partially fails, and analysis text explains any fallbacks. Exceptions from the builder are not
        /// swallowed because they indicate programming or geometry-state issues that should be surfaced during development.
        /// </remarks>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var layerCurves = new List<Curve>();
            var layerPlanes = new List<Plane>();
            Curve? startGuide = null;
            Curve? directionGuide = null;
            double minVoxelSize = 5.0;
            double maxVoxelSize = 15.0;
            double fallbackSpan = 10.0;

            if (!DA.GetDataList(0, layerCurves)) return;
            DA.GetDataList(1, layerPlanes);
            DA.GetData(2, ref startGuide);
            DA.GetData(3, ref directionGuide);
            if (!DA.GetData(4, ref minVoxelSize)) return;
            if (!DA.GetData(5, ref maxVoxelSize)) return;
            if (!DA.GetData(6, ref fallbackSpan)) return;

            if (layerCurves.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "At least one ordered closed layer curve is required.");
                return;
            }

            if (minVoxelSize <= 0.0 || maxVoxelSize <= 0.0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Voxel sizes must be greater than zero.");
                return;
            }

            double tolerance = RhinoDoc.ActiveDoc?.ModelAbsoluteTolerance ?? 0.01;
            ConformalVoxelResult result = ConformalVoxelBuilder.Build(
                layerCurves,
                layerPlanes,
                new ConformalVoxelOptions
                {
                    StartGuideCurve = startGuide,
                    DirectionGuideCurve = directionGuide,
                    MinVoxelSize = minVoxelSize,
                    MaxVoxelSize = maxVoxelSize,
                    FallbackSegmentLength = fallbackSpan
                },
                tolerance);

            DA.SetDataList(0, result.RingCurves);
            DA.SetDataList(1, result.VoxelEdges);
            DA.SetDataList(2, result.VoxelCenters);
            DA.SetDataList(3, result.FaceLoops);
            DA.SetDataList(4, result.Nodes);
            DA.SetData(5, result.SharedPerimeterDivisionCount);
            DA.SetData(6, result.SharedRadialBandCount);
            DA.SetData(7, result.Analysis);

            if (result.VoxelEdges.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No voxel edges were produced. Check that the input curves are closed and ordered as a layer stack.");
            }
        }

        protected override System.Drawing.Bitmap Icon => null!;

        public override Guid ComponentGuid => new Guid("05abdf8f-b24a-4f5a-9236-7bd4a6482ff0");
    }
}
