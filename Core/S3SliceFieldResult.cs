using System.Collections.Generic;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class S3SliceFieldResult
{
    public S3SliceFieldResult(
        IReadOnlyList<Vector3D> optimizedDirections,
        IReadOnlyList<double> scalarValues,
        IReadOnlyList<double> deformationScales,
        IReadOnlyList<double> directionAngles)
    {
        OptimizedDirections = optimizedDirections;
        ScalarValues = scalarValues;
        DeformationScales = deformationScales;
        DirectionAngles = directionAngles;
    }

    public IReadOnlyList<Vector3D> OptimizedDirections { get; }
    public IReadOnlyList<double> ScalarValues { get; }
    public IReadOnlyList<double> DeformationScales { get; }
    public IReadOnlyList<double> DirectionAngles { get; }
}
