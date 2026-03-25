namespace SpatialAdditiveManufacturing.Core.Geometry;

public readonly struct Point3D
{
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

