namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class AxisAngleSet
{
    public AxisAngleSet(double xDegrees, double yDegrees, double zDegrees)
    {
        XDegrees = xDegrees;
        YDegrees = yDegrees;
        ZDegrees = zDegrees;
    }

    public double XDegrees { get; }
    public double YDegrees { get; }
    public double ZDegrees { get; }
}

