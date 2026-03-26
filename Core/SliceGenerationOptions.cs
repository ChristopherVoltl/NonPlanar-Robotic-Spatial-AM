using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Selects which surface-derived direction is used to build the nonplanar field.
/// </summary>
public enum SurfaceFieldMode
{
    SurfaceNormal = 0,
    SurfaceTangent = 1,
}

/// <summary>
/// Controls how extracted output curves are resampled after slicing.
/// </summary>
public enum OutputPolylineSamplingMode
{
    None = 0,
    PointCount = 1,
    MinimumDistance = 2,
}

/// <summary>
/// Groups tuning parameters for the topology-driven nonplanar slicer.
/// </summary>
/// <remarks>
/// This options object exists so the slicer can be driven from Grasshopper without a long method signature.
/// Property values are not automatically validated on assignment; the engine clamps several values at use time.
/// The object is mutable and intended to be configured by the caller before invoking the engine.
/// </remarks>
public sealed class SliceGenerationOptions
{
    /// <summary>
    /// Gets or sets which surface field source is used.
    /// </summary>
    public SurfaceFieldMode FieldMode { get; set; } = SurfaceFieldMode.SurfaceTangent;
    /// <summary>
    /// Gets or sets how final output curves are resampled.
    /// </summary>
    public OutputPolylineSamplingMode OutputSamplingMode { get; set; } = OutputPolylineSamplingMode.MinimumDistance;
    /// <summary>
    /// Gets or sets the percentage of the topology angle that is allowed to influence the slice frame.
    /// Expected range is 0 to 100, but out-of-range values are clamped through <see cref="NormalizedAngleInfluence"/>.
    /// </summary>
    public double AngleInfluencePercent { get; set; } = 100.0;
    /// <summary>
    /// Gets or sets the global up direction used as the planar printing reference.
    /// </summary>
    public Vector3D GlobalUpAxis { get; set; } = Vector3D.ZAxis;
    /// <summary>
    /// Gets or sets smoothing applied when blending adjacent slice orientations. Expected range is 0 to 1.
    /// </summary>
    public double SmoothingFactor { get; set; } = 0.35;

    /// <summary>
    /// Gets or sets the target layer height used when converting normalized scalar changes into layer bands.
    /// Must be greater than zero for meaningful output.
    /// </summary>
    public double TargetLayerHeight { get; set; } = 5.0;
    /// <summary>
    /// Gets or sets the blend between uniform stepping and gradient-adaptive stepping. Expected range is 0 to 1.
    /// </summary>
    public double AdaptiveBlendFactor { get; set; } = 0.5;
    /// <summary>
    /// Gets or sets the minimum normalized scalar step used by the adaptive ordering stage. Must be greater than zero.
    /// </summary>
    public double MinScalarStep { get; set; } = 0.01;
    /// <summary>
    /// Gets or sets the maximum normalized scalar step used by the adaptive ordering stage. Must be greater than or equal to <see cref="MinScalarStep"/>.
    /// </summary>
    public double MaxScalarStep { get; set; } = 0.10;
    /// <summary>
    /// Gets or sets how strongly saddle regions bias the scalar field. Expected range is 0 to 1, though larger values are still accepted.
    /// </summary>
    public double SaddleBiasStrength { get; set; } = 0.35;
    /// <summary>
    /// Gets or sets the falloff exponent used when distributing saddle influence from critical regions.
    /// Must be positive for predictable behavior.
    /// </summary>
    public double SaddleBiasExponent { get; set; } = 1.0;
    /// <summary>
    /// Gets or sets the angular threshold used to classify a section neighborhood as saddle-like.
    /// </summary>
    public double SaddleNormalThresholdDegrees { get; set; } = 40.0;
    /// <summary>
    /// Gets or sets the factor used when smoothing output curves back onto the surface. Expected range is 0 to 1.
    /// </summary>
    public double OutputCurveSmoothingFactor { get; set; } = 0.0;
    /// <summary>
    /// Gets or sets how many smoothing passes are applied to output curves.
    /// </summary>
    public int OutputCurveSmoothingIterations { get; set; } = 5;
    /// <summary>
    /// Gets or sets the requested point count when <see cref="OutputSamplingMode"/> is <see cref="OutputPolylineSamplingMode.PointCount"/>.
    /// </summary>
    public int OutputPolylinePointCount { get; set; } = 0;
    /// <summary>
    /// Gets or sets the requested minimum point spacing when <see cref="OutputSamplingMode"/> is <see cref="OutputPolylineSamplingMode.MinimumDistance"/>.
    /// </summary>
    public double OutputPolylineMinPointDistance { get; set; } = 0.0;
    /// <summary>
    /// Gets or sets the maximum allowed deviation when fitting smoothed or resampled points back to the source surface.
    /// </summary>
    public double SurfaceFollowTolerance { get; set; } = 1.0;

    /// <summary>
    /// Gets the angle influence normalized to the unit interval.
    /// </summary>
    /// <remarks>
    /// Postconditions: callers always receive a value in the inclusive range 0 to 1.
    /// Side-effects: none.
    /// </remarks>
    public double NormalizedAngleInfluence => Clamp(AngleInfluencePercent / 100.0, 0.0, 1.0);

    internal double ClampScalarStep(double step) =>
        Clamp(step, MinScalarStep, MaxScalarStep);

    internal double ClampUnit(double value) =>
        Clamp(value, 0.0, 1.0);

    private static double Clamp(double value, double min, double max)
    {
        if (value < min)
        {
            return min;
        }

        return value > max ? max : value;
    }
}
