namespace SpatialAdditiveManufacturing.Core.Geometry;

public readonly struct LineSegment3D
{
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

