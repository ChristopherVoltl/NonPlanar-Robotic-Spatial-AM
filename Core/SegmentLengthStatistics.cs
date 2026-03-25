namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class SegmentLengthStatistics
{
    public SegmentLengthStatistics(double minimum, double maximum, double average)
    {
        Minimum = minimum;
        Maximum = maximum;
        Average = average;
    }

    public double Minimum { get; }
    public double Maximum { get; }
    public double Average { get; }
}

