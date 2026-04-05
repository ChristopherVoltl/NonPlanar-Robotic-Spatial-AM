using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Grasshopper component that converts sliced perimeter layers into a first-pass spatial lattice scaffold.
    /// </summary>
    /// <remarks>
    /// This component exists to bridge the current nonplanar slicer into spatial-truss design work without
    /// modifying the slicer itself. It focuses on unit-cell generation and graph structure, not Eulerian path planning.
    /// Unexpected behavior to note: the input curve order defines the stacking order from one layer to the next.
    /// Side-effects are limited to Grasshopper runtime messages and output assignment.
    /// </remarks>
    public class SpatialLatticeComponent : GH_Component
    {
        /// <summary>
        /// Initializes the Grasshopper component metadata used for registration and UI display.
        /// </summary>
        public SpatialLatticeComponent()
            : base("Spatial Lattice", "Lattice", "Build a stacked spatial-truss scaffold from ordered nonplanar layer curves.", "FGAM", "SpatialPrinting")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Layer Curves", "C", "Ordered closed layer curves. Input order defines stack order.", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Layer Planes", "P", "Optional representative planes matching the layer order. These help derive inward lattice rings from nonplanar slices.", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager.AddCurveParameter("Seam Guide", "SG", "Optional guide curve used to anchor the startpoint of every normalized layer.", GH_ParamAccess.item);
            pManager[2].Optional = true;
            pManager.AddIntegerParameter("Normalized Divisions", "ND", "Shared perimeter division count used for every layer. Values below 3 use the densest input layer instead.", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Fallback Span", "FS", "Fallback segment length used only when an input layer is not already a usable polyline.", GH_ParamAccess.item, 10.0);
            pManager.AddIntegerParameter("Interior Rings", "IR", "How many inward lattice rings to build from each perimeter layer.", GH_ParamAccess.item, 2);
            pManager.AddNumberParameter("Ring Step Scale", "RSS", "Multiplier applied to the average perimeter segment length to set the inward ring spacing.", GH_ParamAccess.item, 1.0);
            pManager.AddIntegerParameter("Diagonal Mode", "DM", "0 = no diagonals, 1 = alternating diagonals, 2 = cross bracing, 3 = same direction diagonals.", GH_ParamAccess.item, 2);
            pManager.AddCurveParameter("Direction Guide", "DG", "Optional guide curve used to force the point-order direction after the seam is fixed.", GH_ParamAccess.item);
            pManager[8].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Perimeter Rings", "PR", "Expanded perimeter rings used by the lattice graph.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Interior Rings", "IR", "Generated interior rings derived from the perimeter division logic.", GH_ParamAccess.list);
            pManager.AddLineParameter("Lattice Members", "LM", "Stacked truss members including ring edges, spokes, verticals, and diagonals.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Cell Faces", "CF", "Quadrilateral unit-cell faces between adjacent layers.", GH_ParamAccess.list);
            pManager.AddPointParameter("Nodes", "N", "Unique graph nodes for the current lattice scaffold.", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Layer Divisions", "LD", "Base perimeter division counts taken from each input layer.", GH_ParamAccess.list);
            pManager.AddTextParameter("Analysis", "A", "Text summary of the current lattice graph.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var layerCurves = new List<Curve>();
            var layerPlanes = new List<Plane>();
            Curve? seamGuide = null;
            Curve? directionGuide = null;
            int normalizedDivisions = 0;
            double fallbackSpan = 10.0;
            int interiorRings = 2;
            double ringStepScale = 1.0;
            int diagonalMode = 2;

            if (!DA.GetDataList(0, layerCurves)) return;
            DA.GetDataList(1, layerPlanes);
            DA.GetData(2, ref seamGuide);
            if (!DA.GetData(3, ref normalizedDivisions)) return;
            if (!DA.GetData(4, ref fallbackSpan)) return;
            if (!DA.GetData(5, ref interiorRings)) return;
            if (!DA.GetData(6, ref ringStepScale)) return;
            if (!DA.GetData(7, ref diagonalMode)) return;
            DA.GetData(8, ref directionGuide);

            if (layerCurves.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "At least one ordered closed layer curve is required.");
                return;
            }

            double tolerance = RhinoDoc.ActiveDoc?.ModelAbsoluteTolerance ?? 0.01;
            SpatialLatticeDiagonalMode mode = diagonalMode <= 0
                ? SpatialLatticeDiagonalMode.None
                : (diagonalMode == 1
                    ? SpatialLatticeDiagonalMode.Alternating
                    : (diagonalMode == 2 ? SpatialLatticeDiagonalMode.Cross : SpatialLatticeDiagonalMode.SameDirection));

            SpatialLatticeResult result = SpatialLatticeBuilder.Build(
                layerCurves,
                layerPlanes,
                new SpatialLatticeOptions
                {
                    SeamGuideCurve = seamGuide,
                    DirectionGuideCurve = directionGuide,
                    NormalizedDivisionCount = normalizedDivisions,
                    FallbackSegmentLength = fallbackSpan,
                    InteriorRingCount = interiorRings,
                    InteriorStepScale = ringStepScale,
                    DiagonalMode = mode,
                },
                tolerance);

            DA.SetDataList(0, result.PerimeterCurves);
            DA.SetDataList(1, result.InteriorCurves);
            DA.SetDataList(2, result.Members);
            DA.SetDataList(3, result.CellFaces);
            DA.SetDataList(4, result.Nodes);
            DA.SetDataList(5, result.LayerDivisions);
            DA.SetData(6, result.Analysis);

            if (result.Members.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No lattice members were produced. Check that the input curves are closed and ordered as a layer stack.");
            }
        }

        protected override System.Drawing.Bitmap Icon => null!;

        public override Guid ComponentGuid => new Guid("1b06de5e-37aa-47d0-b8a3-7b428d30842d");
    }
}
