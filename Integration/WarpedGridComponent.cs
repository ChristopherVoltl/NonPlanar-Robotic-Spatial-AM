using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Grasshopper component that builds a square warped-grid quad mesh on each nonplanar layer.
    /// </summary>
    /// <remarks>
    /// The component starts from a square grid in parameter space and morphs it to each normalized layer perimeter.
    /// It exists to provide a quad-mesh-based interior scaffold that can later inform spatial toolpaths. Side-effects
    /// are limited to Grasshopper runtime messages and output assignment.
    /// </remarks>
    public class WarpedGridComponent : GH_Component
    {
        public WarpedGridComponent()
            : base("Warped Grid", "WGrid", "Generate a warped square-grid quad mesh that conforms to ordered nonplanar layers.", "FGAM", "SpatialPrinting")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Layer Curves", "C", "Ordered closed layer curves. Input order defines stack order.", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Layer Planes", "P", "Optional representative planes matching the layer order.", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager.AddCurveParameter("Start Guide", "SG", "Optional guide curve used to anchor the startpoint of every normalized layer.", GH_ParamAccess.item);
            pManager[2].Optional = true;
            pManager.AddCurveParameter("Direction Guide", "DG", "Optional guide curve used to bias the forward direction after the seam is fixed.", GH_ParamAccess.item);
            pManager[3].Optional = true;
            pManager.AddIntegerParameter("Grid Resolution", "R", "Number of quads per side of the base square grid.", GH_ParamAccess.item, 8);
            pManager.AddNumberParameter("Adaptive Strength", "AS", "How strongly samples redistribute inside already-refined regions. 0 keeps spacing uniform after subdivision.", GH_ParamAccess.item, 0.35);
            pManager.AddNumberParameter("Deviation Threshold", "DT", "Only regions whose boundary approximation error exceeds this threshold are subdivided.", GH_ParamAccess.item, 1.0);
            pManager.AddIntegerParameter("Adaptive Levels", "AL", "How many recursive refinement passes can insert extra rows and columns.", GH_ParamAccess.item, 2);
            pManager.AddIntegerParameter("Adaptive Smooth", "SM", "How many smoothing passes are applied to the adaptive boundary weights.", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("Fallback Span", "FS", "Fallback segment length used only when an input layer is not already a usable polyline.", GH_ParamAccess.item, 10.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Layer Meshes", "M", "Warped quad meshes, one per layer.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Grid Curves", "G", "Per-layer grid polylines for rows and columns.", GH_ParamAccess.list);
            pManager.AddLineParameter("Inter-Layer Struts", "S", "Lines connecting matching grid nodes between adjacent layers.", GH_ParamAccess.list);
            pManager.AddPointParameter("Nodes", "N", "Unique grid nodes across the layer stack.", GH_ParamAccess.list);
            pManager.AddPointParameter("Cell Centers", "CC", "Centers of stacked quad cells between adjacent layers.", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Boundary Divisions", "BD", "Shared boundary division count used to normalize every layer.", GH_ParamAccess.item);
            pManager.AddTextParameter("Analysis", "A", "Diagnostic summary of the warped-grid solve.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var layerCurves = new List<Curve>();
            var layerPlanes = new List<Plane>();
            Curve? startGuide = null;
            Curve? directionGuide = null;
            int gridResolution = 8;
            double adaptiveStrength = 0.35;
            double deviationThreshold = 1.0;
            int adaptiveLevels = 2;
            int adaptiveSmooth = 1;
            double fallbackSpan = 10.0;

            if (!DA.GetDataList(0, layerCurves)) return;
            DA.GetDataList(1, layerPlanes);
            DA.GetData(2, ref startGuide);
            DA.GetData(3, ref directionGuide);
            if (!DA.GetData(4, ref gridResolution)) return;
            if (!DA.GetData(5, ref adaptiveStrength)) return;
            if (!DA.GetData(6, ref deviationThreshold)) return;
            if (!DA.GetData(7, ref adaptiveLevels)) return;
            if (!DA.GetData(8, ref adaptiveSmooth)) return;
            if (!DA.GetData(9, ref fallbackSpan)) return;

            if (layerCurves.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "At least one ordered closed layer curve is required.");
                return;
            }

            if (gridResolution < 1)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Grid Resolution must be at least 1.");
                return;
            }

            double tolerance = RhinoDoc.ActiveDoc?.ModelAbsoluteTolerance ?? 0.01;
            WarpedGridResult result = WarpedGridBuilder.Build(
                layerCurves,
                layerPlanes,
                new WarpedGridOptions
                {
                    StartGuideCurve = startGuide,
                    DirectionGuideCurve = directionGuide,
                    GridResolution = gridResolution,
                    AdaptiveStrength = adaptiveStrength,
                    AdaptiveDeviationThreshold = deviationThreshold,
                    AdaptiveLevels = adaptiveLevels,
                    AdaptiveSmoothingPasses = adaptiveSmooth,
                    FallbackSegmentLength = fallbackSpan
                },
                tolerance);

            DA.SetDataList(0, result.LayerMeshes);
            DA.SetDataList(1, result.GridCurves);
            DA.SetDataList(2, result.InterLayerStruts);
            DA.SetDataList(3, result.Nodes);
            DA.SetDataList(4, result.CellCenters);
            DA.SetData(5, result.BoundaryDivisionCount);
            DA.SetData(6, result.Analysis);

            if (result.LayerMeshes.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No warped grids were produced. Check that the input curves are closed and ordered as a layer stack.");
            }
        }

        protected override System.Drawing.Bitmap Icon => null!;

        public override Guid ComponentGuid => new Guid("4f5b8c93-85bc-4d14-9201-fd560363727b");
    }
}
