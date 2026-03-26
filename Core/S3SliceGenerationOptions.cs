using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Groups the parameters for the S3-inspired deformation-based slicer.
/// </summary>
/// <remarks>
/// These options are consumed by the Rhino-side S3 workflow, which approximates an inner orientation-field solve and an
/// outer deformation/scalar-field solve. Values are not validated on assignment; the engine clamps several derived quantities.
/// </remarks>
public sealed class S3SliceGenerationOptions
{
    /// <summary>
    /// Gets or sets the nominal distance between layers. Must be greater than zero for meaningful output.
    /// </summary>
    public double LayerHeight { get; set; } = 5.0;
    /// <summary>
    /// Gets or sets the number of orientation-field smoothing iterations. Values less than one are effectively treated as one.
    /// </summary>
    public int InnerIterations { get; set; } = 16;
    /// <summary>
    /// Gets or sets the number of outer deformation/scalar iterations. Values less than one are effectively treated as one.
    /// </summary>
    public int OuterIterations { get; set; } = 10;
    /// <summary>
    /// Gets or sets the neighbor weight used during quaternion-style field averaging. Expected range is 0 to 1.
    /// </summary>
    public double QuaternionSmoothness { get; set; } = 0.45;
    /// <summary>
    /// Gets or sets the overall strength of nonplanar deformation. Expected range is 0 to 1.
    /// </summary>
    public double DeformationStrength { get; set; } = 0.35;
    /// <summary>
    /// Gets or sets the weight of the support-related objective in the print-direction field.
    /// </summary>
    public double SupportFreeWeight { get; set; } = 1.0;
    /// <summary>
    /// Gets or sets the weight of the strength-related objective in the print-direction field.
    /// </summary>
    public double StrengthWeight { get; set; } = 0.75;
    /// <summary>
    /// Gets or sets the weight of the surface-quality objective in the print-direction field.
    /// </summary>
    public double SurfaceQualityWeight { get; set; } = 0.75;
    /// <summary>
    /// Gets or sets the global planar reference direction, typically world Z.
    /// </summary>
    public Vector3D GlobalUpAxis { get; set; } = Vector3D.ZAxis;
    /// <summary>
    /// Gets or sets the curve smoothing factor applied after iso-curve extraction. Expected range is 0 to 1.
    /// </summary>
    public double OutputCurveSmoothingFactor { get; set; } = 0.0;
    /// <summary>
    /// Gets or sets how many output smoothing passes are applied.
    /// </summary>
    public int OutputCurveSmoothingIterations { get; set; } = 5;
    /// <summary>
    /// Gets or sets the post-extraction resampling mode for output curves.
    /// </summary>
    public OutputPolylineSamplingMode OutputSamplingMode { get; set; } = OutputPolylineSamplingMode.None;
    /// <summary>
    /// Gets or sets the target point count used when point-count resampling is active.
    /// </summary>
    public int OutputPolylinePointCount { get; set; } = 0;
    /// <summary>
    /// Gets or sets the minimum point spacing used when minimum-distance resampling is active.
    /// </summary>
    public double OutputPolylineMinPointDistance { get; set; } = 0.0;
    /// <summary>
    /// Gets or sets the allowed deviation when projecting post-processed curves back to the Brep.
    /// </summary>
    public double SurfaceFollowTolerance { get; set; } = 1.0;

    internal double ClampUnit(double value)
    {
        if (value < 0.0)
        {
            return 0.0;
        }

        return value > 1.0 ? 1.0 : value;
    }
}
