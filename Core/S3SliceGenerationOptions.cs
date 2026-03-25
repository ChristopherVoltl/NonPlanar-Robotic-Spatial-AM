using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class S3SliceGenerationOptions
{
    public double LayerHeight { get; set; } = 5.0;
    public int InnerIterations { get; set; } = 16;
    public int OuterIterations { get; set; } = 10;
    public double QuaternionSmoothness { get; set; } = 0.45;
    public double DeformationStrength { get; set; } = 0.35;
    public double SupportFreeWeight { get; set; } = 1.0;
    public double StrengthWeight { get; set; } = 0.75;
    public double SurfaceQualityWeight { get; set; } = 0.75;
    public Vector3D GlobalUpAxis { get; set; } = Vector3D.ZAxis;
    public double OutputCurveSmoothingFactor { get; set; } = 0.0;
    public int OutputCurveSmoothingIterations { get; set; } = 5;
    public OutputPolylineSamplingMode OutputSamplingMode { get; set; } = OutputPolylineSamplingMode.None;
    public int OutputPolylinePointCount { get; set; } = 0;
    public double OutputPolylineMinPointDistance { get; set; } = 0.0;
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
