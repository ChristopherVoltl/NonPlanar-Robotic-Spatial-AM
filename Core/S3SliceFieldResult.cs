using System.Collections.Generic;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Captures the intermediate field output of the S3-inspired solver.
/// </summary>
/// <remarks>
/// This result exists because the S3 Rhino workflow separates field generation from actual iso-curve extraction.
/// Callers typically pass this data into Rhino-side helpers rather than consuming it directly as printable geometry.
/// </remarks>
public sealed class S3SliceFieldResult
{
    /// <summary>
    /// Initializes a new field result.
    /// </summary>
    /// <remarks>
    /// Preconditions: list arguments should not be <see langword="null"/> and should all represent the same node ordering.
    /// Postconditions: references are stored exactly as supplied.
    /// Exceptions: none in this constructor.
    /// Side-effects: none.
    /// </remarks>
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
