namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Stores the angles, in degrees, between a slice normal and the global X, Y, and Z axes.
/// </summary>
/// <remarks>
/// This type exists so callers can inspect machine-relevant orientation values without recalculating them from the slice normal.
/// </remarks>
public sealed class AxisAngleSet
{
    /// <summary>
    /// Initializes a new axis-angle set.
    /// </summary>
    /// <remarks>
    /// Preconditions: callers should pass values in degrees for the same normal/orientation frame.
    /// Postconditions: values are stored as supplied.
    /// Exceptions: none.
    /// Side-effects: none.
    /// </remarks>
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

