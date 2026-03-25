using System.Collections.Generic;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class SlicePath
{
    public SlicePath(int index, TopologySection sourceSection, SliceFrame frame, IReadOnlyList<LineSegment3D> segments)
    {
        Index = index;
        SourceSection = sourceSection;
        Frame = frame;
        Segments = segments;
    }

    public int Index { get; }
    public TopologySection SourceSection { get; }
    public SliceFrame Frame { get; }
    public IReadOnlyList<LineSegment3D> Segments { get; }
}

