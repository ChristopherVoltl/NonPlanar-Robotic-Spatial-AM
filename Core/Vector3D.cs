using System;

namespace SpatialAdditiveManufacturing.Core.Geometry;

public readonly struct Vector3D
{
    public static readonly Vector3D Zero = new(0.0, 0.0, 0.0);
    public static readonly Vector3D XAxis = new(1.0, 0.0, 0.0);
    public static readonly Vector3D YAxis = new(0.0, 1.0, 0.0);
    public static readonly Vector3D ZAxis = new(0.0, 0.0, 1.0);

    public Vector3D(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    public double X { get; }
    public double Y { get; }
    public double Z { get; }

    public double Length => Math.Sqrt((X * X) + (Y * Y) + (Z * Z));

    public Vector3D Unitized()
    {
        var length = Length;
        if (length <= NumericalTolerance.Epsilon)
        {
            return ZAxis;
        }

        return new Vector3D(X / length, Y / length, Z / length);
    }

    public double Dot(Vector3D other) => (X * other.X) + (Y * other.Y) + (Z * other.Z);

    public Vector3D Cross(Vector3D other) =>
        new(
            (Y * other.Z) - (Z * other.Y),
            (Z * other.X) - (X * other.Z),
            (X * other.Y) - (Y * other.X));

    public double AngleTo(Vector3D other)
    {
        var a = Unitized();
        var b = other.Unitized();
        var dot = Math.Max(-1.0, Math.Min(1.0, a.Dot(b)));
        return Math.Acos(dot) * (180.0 / Math.PI);
    }

    public static Vector3D Lerp(Vector3D start, Vector3D end, double t)
    {
        var clamped = Math.Max(0.0, Math.Min(1.0, t));
        return new Vector3D(
            start.X + ((end.X - start.X) * clamped),
            start.Y + ((end.Y - start.Y) * clamped),
            start.Z + ((end.Z - start.Z) * clamped)).Unitized();
    }

    public static Vector3D operator +(Vector3D left, Vector3D right) =>
        new(left.X + right.X, left.Y + right.Y, left.Z + right.Z);

    public static Vector3D operator -(Vector3D left, Vector3D right) =>
        new(left.X - right.X, left.Y - right.Y, left.Z - right.Z);

    public static Vector3D operator /(Vector3D vector, double scalar) =>
        new(vector.X / scalar, vector.Y / scalar, vector.Z / scalar);

    public static Vector3D operator *(Vector3D vector, double scalar) =>
        new(vector.X * scalar, vector.Y * scalar, vector.Z * scalar);
}

