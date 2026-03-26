namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Stores minimum, maximum, and average angle values in degrees.
/// </summary>
public sealed class AngleStatistics
{
    /// <summary>
    /// Initializes a set of angle statistics.
    /// </summary>
    /// <remarks>
    /// Preconditions: callers should supply values expressed in degrees and ensure the minimum/maximum/average relationship is meaningful.
    /// Postconditions: values are stored as supplied.
    /// Exceptions: none.
    /// Side-effects: none.
    /// </remarks>
    public AngleStatistics(double minimum, double maximum, double average)
    {
        Minimum = minimum;
        Maximum = maximum;
        Average = average;
    }

    public double Minimum { get; }
    public double Maximum { get; }
    public double Average { get; }
}

