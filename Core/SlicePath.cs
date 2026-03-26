using System.Collections.Generic;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Represents one generated slice path together with the topology section that produced it.
/// </summary>
/// <remarks>
/// This class exists so callers can retain both the machine-facing segments and the source topology context.
/// Instances are immutable and have no side-effects after construction.
/// </remarks>
public sealed class SlicePath
{
    /// <summary>
    /// Initializes a slice path record.
    /// </summary>
    /// <param name="index">The final slice order index.</param>
    /// <param name="sourceSection">The topology section that was converted into this slice.</param>
    /// <param name="frame">The representative frame for this slice.</param>
    /// <param name="segments">The ordered printable path segments for the slice.</param>
    /// <remarks>
    /// Preconditions: <paramref name="sourceSection"/>, <paramref name="frame"/>, and <paramref name="segments"/> should not be <see langword="null"/>.
    /// Postconditions: references are stored exactly as supplied.
    /// Exceptions: none in this constructor; later callers may fail if required arguments were <see langword="null"/>.
    /// Side-effects: none.
    /// </remarks>
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

