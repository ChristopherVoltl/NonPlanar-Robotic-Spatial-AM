using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class SliceFrame
{
    public SliceFrame(Point3D origin, Vector3D normal, double angleFromGlobalUpDegrees, AxisAngleSet axisAngles)
    {
        Origin = origin;
        Normal = normal.Unitized();
        AngleFromGlobalUpDegrees = angleFromGlobalUpDegrees;
        AxisAngles = axisAngles;
    }

    public Point3D Origin { get; }
    public Vector3D Normal { get; }
    public double AngleFromGlobalUpDegrees { get; }
    public AxisAngleSet AxisAngles { get; }
}

