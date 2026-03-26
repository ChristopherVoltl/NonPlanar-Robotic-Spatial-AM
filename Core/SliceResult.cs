using System.Collections.Generic;

namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Represents the full output of a slicing run.
/// </summary>
/// <remarks>
/// The result object exists so callers can consume the generated paths and their analytics together.
/// It is immutable and does not own external resources.
/// </remarks>
public sealed class SliceResult
{
    /// <summary>
    /// Initializes a new slice result.
    /// </summary>
    /// <param name="slices">The generated slice paths in final order.</param>
    /// <param name="analytics">Summary analytics for the same slice set.</param>
    /// <remarks>
    /// Preconditions: arguments should not be <see langword="null"/>.
    /// Postconditions: references are stored exactly as supplied.
    /// Exceptions: none in this constructor.
    /// Side-effects: none.
    /// </remarks>
    public SliceResult(IReadOnlyList<SlicePath> slices, SliceAnalytics analytics)
    {
        Slices = slices;
        Analytics = analytics;
    }

    public IReadOnlyList<SlicePath> Slices { get; }
    public SliceAnalytics Analytics { get; }
}

