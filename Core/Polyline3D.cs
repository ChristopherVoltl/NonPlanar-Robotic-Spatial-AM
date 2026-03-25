using System.Collections.Generic;
using System.Linq;

namespace SpatialAdditiveManufacturing.Core.Geometry;

public sealed class Polyline3D
{
    public Polyline3D(IEnumerable<Point3D> points)
    {
        Points = points?.ToList() ?? new List<Point3D>();
    }

    public IReadOnlyList<Point3D> Points { get; }

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

