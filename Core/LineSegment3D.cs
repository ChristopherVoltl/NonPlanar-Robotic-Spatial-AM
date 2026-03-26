namespace SpatialAdditiveManufacturing.Core.Geometry;

/// <summary>
/// Represents a directed line segment between two 3D points.
/// </summary>
/// <remarks>
/// This type is used to expose printable path fragments without depending on RhinoCommon curve types.
/// The struct is immutable and has no side-effects.
/// </remarks>
public readonly struct LineSegment3D
{
    /// <summary>
    /// Initializes a new segment.
    /// </summary>
    /// <param name="start">The start point.</param>
    /// <param name="end">The end point.</param>
    /// <remarks>
    /// Preconditions: callers should provide points in the same coordinate system and model units.
    /// Postconditions: the segment endpoints are stored exactly as supplied.
    /// Exceptions: none.
    /// Side-effects: none.
    /// </remarks>
    public LineSegment3D(Point3D start, Point3D end)
    {
        Start = start;
        End = end;
    }

    public Point3D Start { get; }
    public Point3D End { get; }
    public double Length => (End - Start).Length;
    public Vector3D Direction => (End - Start).Unitized();
}

