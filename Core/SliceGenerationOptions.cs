using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public enum SurfaceFieldMode
{
    SurfaceNormal = 0,
    SurfaceTangent = 1,
}

public enum OutputPolylineSamplingMode
{
    None = 0,
    PointCount = 1,
    MinimumDistance = 2,
}

public sealed class SliceGenerationOptions
{
    public SurfaceFieldMode FieldMode { get; set; } = SurfaceFieldMode.SurfaceTangent;
    public OutputPolylineSamplingMode OutputSamplingMode { get; set; } = OutputPolylineSamplingMode.MinimumDistance;
    public double AngleInfluencePercent { get; set; } = 100.0;
    public Vector3D GlobalUpAxis { get; set; } = Vector3D.ZAxis;
    public double SmoothingFactor { get; set; } = 0.35;

    // Paper-inspired adaptive slicing controls.
    public double TargetLayerHeight { get; set; } = 5.0;
    public double AdaptiveBlendFactor { get; set; } = 0.5;
    public double MinScalarStep { get; set; } = 0.01;
    public double MaxScalarStep { get; set; } = 0.10;
    public double SaddleBiasStrength { get; set; } = 0.35;
    public double SaddleBiasExponent { get; set; } = 1.0;
    public double SaddleNormalThresholdDegrees { get; set; } = 40.0;
    public double OutputCurveSmoothingFactor { get; set; } = 0.0;
    public int OutputCurveSmoothingIterations { get; set; } = 5;
    public int OutputPolylinePointCount { get; set; } = 0;
    public double OutputPolylineMinPointDistance { get; set; } = 0.0;
    public double SurfaceFollowTolerance { get; set; } = 1.0;

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
