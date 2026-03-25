using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace NonPlanar_Robotic_Spatial_AM
{
    public class NonPlanar_Robotic_Spatial_AMInfo : GH_AssemblyInfo
    {
        public override string Name => "NonPlanar Robotic Spatial AM";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => null!;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "";

        public override Guid Id => new Guid("869fc472-2341-4fe0-95ed-ffa724afed41");

        //Return a string identifying you or your company.
        public override string AuthorName => "";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "";
    }
}
