using System.Collections.Generic;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class S3NodeSample
{
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
