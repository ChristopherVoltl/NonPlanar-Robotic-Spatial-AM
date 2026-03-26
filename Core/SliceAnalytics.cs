namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Aggregates the most important measurements for a slice result.
/// </summary>
/// <remarks>
/// These values are intended for diagnostics, UI display, and toolpath validation rather than further geometric computation.
/// The class is immutable and has no side-effects.
/// </remarks>
public sealed class SliceAnalytics
{
    /// <summary>
    /// Initializes the analytics container.
    /// </summary>
    /// <remarks>
    /// Preconditions: callers should provide already-computed statistics objects that are not <see langword="null"/>.
    /// Postconditions: references are stored exactly as supplied.
    /// Exceptions: none in this constructor.
    /// Side-effects: none.
    /// </remarks>
    public SliceAnalytics(
        SegmentLengthStatistics segmentLengths,
        AngleStatistics sliceAnglesFromGlobalUp,
        AngleStatistics xAxisAngles,
        AngleStatistics yAxisAngles,
        AngleStatistics zAxisAngles)
    {
        SegmentLengths = segmentLengths;
        SliceAnglesFromGlobalUp = sliceAnglesFromGlobalUp;
        XAxisAngles = xAxisAngles;
        YAxisAngles = yAxisAngles;
        ZAxisAngles = zAxisAngles;
    }

    public SegmentLengthStatistics SegmentLengths { get; }
    public AngleStatistics SliceAnglesFromGlobalUp { get; }
    public AngleStatistics XAxisAngles { get; }
    public AngleStatistics YAxisAngles { get; }
    public AngleStatistics ZAxisAngles { get; }
}

