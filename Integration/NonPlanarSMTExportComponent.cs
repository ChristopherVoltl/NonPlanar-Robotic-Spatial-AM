using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Grasshopper component responsible only for:
    /// sliced curves -> SMT export.
    /// </summary>
    public class NonPlanarSMTExportComponent : GH_Component
    {
        public NonPlanarSMTExportComponent()
            : base("SMT Export", "SMT X", "Translate sliced curves into SMT point data.", "FGAM", "SpatialPrinting")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Path Curves", "C", "Sliced curves to export to SMT.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Vertical_E5", "V_E5", "Extrusion value used for vertical segments.", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Angled_E5", "A_E5", "Extrusion value used for angled segments.", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Horizontal_E5", "H_E5", "Extrusion value used for horizontal segments.", GH_ParamAccess.item, 1.4);
            pManager.AddNumberParameter("Velocity Ratio Multiplier", "VRx", "Multiplier used to scale travel and print velocities.", GH_ParamAccess.item, 1.4);
            pManager.AddBooleanParameter("Run", "R", "If true, write the current curves into SMT.", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "P", "Generated SMT path planes.", GH_ParamAccess.list);
            pManager.AddTextParameter("Log", "L", "Status message from the SMT writer.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var curves = new List<Curve>();
            double verticalE5 = 1.4;
            double angledE5 = 1.4;
            double horizontalE5 = 1.4;
            double velocityRatioMultiplier = 1.4;
            bool run = false;

            if (!DA.GetDataList(0, curves)) return;
            if (!DA.GetData(1, ref verticalE5)) return;
            if (!DA.GetData(2, ref angledE5)) return;
            if (!DA.GetData(3, ref horizontalE5)) return;
            if (!DA.GetData(4, ref velocityRatioMultiplier)) return;
            if (!DA.GetData(5, ref run)) return;

            if (!run)
            {
                DA.SetData(1, "Waiting for Run = true.");
                return;
            }

            var result = SMTConnectionWriter.WriteAllToSMT(
                curves,
                verticalE5,
                angledE5,
                horizontalE5,
                velocityRatioMultiplier);

            DA.SetDataList(0, result.GeneratedPlanes);
            DA.SetData(1, result.LogMessage);
        }

        protected override System.Drawing.Bitmap Icon => null!;

        public override Guid ComponentGuid => new Guid("bd12d26f-4b90-48e8-b6bf-e82c2d7ff948");
    }
}

