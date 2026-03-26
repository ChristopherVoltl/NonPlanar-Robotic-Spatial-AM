using System.Collections.Generic;
using System.Linq;

namespace SpatialAdditiveManufacturing.Core.Geometry;

/// <summary>
/// Represents an ordered list of 3D points that define a polyline guide path.
/// </summary>
/// <remarks>
/// This exists to keep the core slicers independent from Rhino's polyline types while still carrying path topology.
/// Instances are immutable after construction.
/// </remarks>
public sealed class Polyline3D
{
    /// <summary>
    /// Initializes a new polyline from a point sequence.
    /// </summary>
    /// <param name="points">
    /// The ordered points that define the guide path. The value may be <see langword="null"/>, in which case an empty polyline is created.
    /// </param>
    /// <remarks>
    /// Preconditions: callers should supply points in path order and in a consistent unit system.
    /// Postconditions: the input sequence is materialized into an internal read-only list.
    /// Exceptions: none. A <see langword="null"/> sequence is converted to an empty polyline.
    /// Side-effects: the input sequence is enumerated once.
    /// </remarks>
    public Polyline3D(IEnumerable<Point3D> points)
    {
        Points = points?.ToList() ?? new List<Point3D>();
    }

    public IReadOnlyList<Point3D> Points { get; }

    /// <summary>
    /// Converts the stored points into contiguous line segments.
    /// </summary>
    /// <returns>
    /// A list containing one segment for every adjacent point pair. If fewer than two points are present, an empty list is returned.
    /// </returns>
    /// <remarks>
    /// Preconditions: none.
    /// Postconditions: the returned list is newly allocated and does not share mutable state with this instance.
    /// Exceptions: none.
    /// Differences: this method preserves the existing point order and does not attempt to close the polyline automatically.
    /// Side-effects: allocates a new list.
    /// </remarks>
    public IReadOnlyList<LineSegment3D> ToSegments()
    {
        var segments = new List<LineSegment3D>();
        for (var i = 0; i < Points.Count - 1; i++)
        {
            segments.Add(new LineSegment3D(Points[i], Points[i + 1]));
        }

        return segments;
    }
}

