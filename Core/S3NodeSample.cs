using System.Collections.Generic;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Represents one sampled node in the S3-style field graph.
/// </summary>
/// <remarks>
/// This type exists because the S3-inspired solver operates on sampled graph nodes rather than directly on Breps or curves.
/// The stored directions capture domain concepts that are not self-evident outside this project:
/// surface normal is the local surface orientation, support direction is a printing-support proxy, and curvature direction
/// is a tangent-like proxy used to bias the print-direction field.
/// </remarks>
public sealed class S3NodeSample
{
    /// <summary>
    /// Initializes a sampled S3 node.
    /// </summary>
    /// <remarks>
    /// Preconditions: callers should provide a consistent coordinate frame, a neighborhood index list that refers to valid nodes,
    /// and meaningful local directions.
    /// Postconditions: the three direction vectors are unitized before storage.
    /// Exceptions: none in this constructor; later algorithms may fail if neighbors reference invalid indices.
    /// Side-effects: none.
    /// </remarks>
    public S3NodeSample(
        int index,
        Point3D position,
        Vector3D surfaceNormal,
        Vector3D supportDirection,
        Vector3D curvatureDirection,
        IReadOnlyList<int> neighbors)
    {
        Index = index;
        Position = position;
        SurfaceNormal = surfaceNormal.Unitized();
        SupportDirection = supportDirection.Unitized();
        CurvatureDirection = curvatureDirection.Unitized();
        Neighbors = neighbors;
    }

    public int Index { get; }
    public Point3D Position { get; }
    public Vector3D SurfaceNormal { get; }
    public Vector3D SupportDirection { get; }
    public Vector3D CurvatureDirection { get; }
    public IReadOnlyList<int> Neighbors { get; }
}
