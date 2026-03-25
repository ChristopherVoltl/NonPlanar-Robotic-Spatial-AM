using System.Collections.Generic;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class SliceResult
{
    public SliceResult(IReadOnlyList<SlicePath> slices, SliceAnalytics analytics)
    {
        Slices = slices;
        Analytics = analytics;
    }

    public IReadOnlyList<SlicePath> Slices { get; }
    public SliceAnalytics Analytics { get; }
}

