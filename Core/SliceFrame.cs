using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Describes the orientation and placement of one generated slice layer.
/// </summary>
/// <remarks>
/// The frame is the bridge between geometric analysis and machine-facing toolpath planning. It stores the layer origin,
/// the slice normal, and convenience angle measurements relative to the global coordinate system.
/// </remarks>
public sealed class SliceFrame
{
    /// <summary>
    /// Initializes a slice frame.
    /// </summary>
    /// <param name="origin">The representative point for the slice.</param>
    /// <param name="normal">The slice normal. It is unitized before storage.</param>
    /// <param name="angleFromGlobalUpDegrees">The angle from the configured global up axis, in degrees.</param>
    /// <param name="axisAngles">Per-axis angle measurements used for analysis output.</param>
    /// <remarks>
    /// Preconditions: callers should provide a meaningful normal and angle values that correspond to the same coordinate frame.
    /// Postconditions: <paramref name="normal"/> is normalized before storage.
    /// Exceptions: none.
    /// Side-effects: none.
    /// </remarks>
    public SliceFrame(Point3D origin, Vector3D normal, double angleFromGlobalUpDegrees, AxisAngleSet axisAngles)
    {
        Origin = origin;
        Normal = normal.Unitized();
        AngleFromGlobalUpDegrees = angleFromGlobalUpDegrees;
        AxisAngles = axisAngles;
    }

    public Point3D Origin { get; }
    public Vector3D Normal { get; }
    public double AngleFromGlobalUpDegrees { get; }
    public AxisAngleSet AxisAngles { get; }
}

