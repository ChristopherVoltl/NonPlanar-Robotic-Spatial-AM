namespace SpatialAdditiveManufacturing.Core.Geometry;

/// <summary>
/// Represents a point in the slicer's geometry space.
/// </summary>
/// <remarks>
/// This type exists so the core slicing engines can operate without taking a direct dependency on Rhino types.
/// All coordinates are assumed to be expressed in the caller's model units.
/// The struct is immutable and has no side-effects.
/// </remarks>
public readonly struct Point3D
{
    /// <summary>
    /// Initializes a new point value.
    /// </summary>
    /// <param name="x">The X coordinate in model units. Any finite value is allowed.</param>
    /// <param name="y">The Y coordinate in model units. Any finite value is allowed.</param>
    /// <param name="z">The Z coordinate in model units. Any finite value is allowed.</param>
    /// <remarks>
    /// Preconditions: callers should pass finite coordinates that use the same unit system as the rest of the slice data.
    /// Postconditions: the created instance stores the supplied coordinates without normalization or validation.
    /// Exceptions: this constructor does not throw by itself.
    /// Side-effects: none.
    /// </remarks>
    public Point3D(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    public double X { get; }
    public double Y { get; }
    public double Z { get; }

    public static Vector3D operator -(Point3D end, Point3D start) =>
        new(end.X - start.X, end.Y - start.Y, end.Z - start.Z);

    public static Point3D operator +(Point3D point, Vector3D vector) =>
        new(point.X + vector.X, point.Y + vector.Y, point.Z + vector.Z);
}

