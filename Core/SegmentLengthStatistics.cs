namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Stores minimum, maximum, and average segment lengths for a slice set.
/// </summary>
public sealed class SegmentLengthStatistics
{
    /// <summary>
    /// Initializes a set of segment-length statistics.
    /// </summary>
    /// <remarks>
    /// Preconditions: callers should provide values in model units and ensure the minimum/maximum/average relationship is meaningful.
    /// Postconditions: values are stored as supplied.
    /// Exceptions: none.
    /// Side-effects: none.
    /// </remarks>
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

