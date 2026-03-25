namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class SliceAnalytics
{
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

