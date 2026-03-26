using System;

namespace SpatialAdditiveManufacturing.Core.Geometry;

/// <summary>
/// Represents a 3D direction or displacement used by the slicing algorithms.
/// </summary>
/// <remarks>
/// This type exists so the core library can express vector math independently of RhinoCommon.
/// The struct is immutable and all methods return new values rather than mutating global or shared state.
/// </remarks>
public readonly struct Vector3D
{
    public static readonly Vector3D Zero = new(0.0, 0.0, 0.0);
    public static readonly Vector3D XAxis = new(1.0, 0.0, 0.0);
    public static readonly Vector3D YAxis = new(0.0, 1.0, 0.0);
    public static readonly Vector3D ZAxis = new(0.0, 0.0, 1.0);

    /// <summary>
    /// Initializes a new vector value.
    /// </summary>
    /// <param name="x">The X component. Any finite value is allowed.</param>
    /// <param name="y">The Y component. Any finite value is allowed.</param>
    /// <param name="z">The Z component. Any finite value is allowed.</param>
    /// <remarks>
    /// Preconditions: callers should provide finite components expressed in model units.
    /// Postconditions: the vector is stored exactly as supplied and is not normalized.
    /// Exceptions: this constructor does not throw by itself.
    /// Side-effects: none.
    /// </remarks>
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

    /// <summary>
    /// Returns the unit-length version of this vector.
    /// </summary>
    /// <returns>
    /// A normalized vector. If the source vector is effectively zero-length, <see cref="ZAxis"/> is returned as a stable fallback.
    /// </returns>
    /// <remarks>
    /// This method exists because many slicing operations depend on stable normals and directions.
    /// Preconditions: none.
    /// Postconditions: the returned vector has unit length unless the zero-vector fallback was required.
    /// Exceptions: none.
    /// Differences: unlike some vector libraries, this method never throws on zero length and never mutates the current value.
    /// Side-effects: none.
    /// </remarks>
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

    /// <summary>
    /// Computes the angle to another vector in degrees.
    /// </summary>
    /// <param name="other">The comparison vector. Zero-length inputs are tolerated and fall back through <see cref="Unitized"/>.</param>
    /// <returns>An angle in the inclusive range 0 to 180 degrees.</returns>
    /// <remarks>
    /// Preconditions: none.
    /// Postconditions: callers can safely treat the result as degrees measured between normalized directions.
    /// Exceptions: none.
    /// Side-effects: none.
    /// </remarks>
    public double AngleTo(Vector3D other)
    {
        var a = Unitized();
        var b = other.Unitized();
        var dot = Math.Max(-1.0, Math.Min(1.0, a.Dot(b)));
        return Math.Acos(dot) * (180.0 / Math.PI);
    }

    /// <summary>
    /// Blends between two directions and normalizes the result.
    /// </summary>
    /// <param name="start">The first direction.</param>
    /// <param name="end">The second direction.</param>
    /// <param name="t">The interpolation factor. Values outside 0 to 1 are clamped.</param>
    /// <returns>A normalized blended direction.</returns>
    /// <remarks>
    /// This helper exists because the slicers frequently transition between global and topology-driven orientations.
    /// Preconditions: none.
    /// Postconditions: the returned vector is unitized and <paramref name="t"/> is effectively clamped to the unit interval.
    /// Exceptions: none.
    /// Side-effects: none.
    /// </remarks>
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

