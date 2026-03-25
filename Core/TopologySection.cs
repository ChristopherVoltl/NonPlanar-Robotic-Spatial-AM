using System.Collections.Generic;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public sealed class TopologySection
{
    public TopologySection(
        int index,
        Polyline3D guidePath,
        Vector3D topologyNormal,
        IReadOnlyDictionary<string, double>? metadata = null)
    {
        Index = index;
        GuidePath = guidePath;
        TopologyNormal = topologyNormal.Unitized();
        Metadata = metadata;
    }

    public int Index { get; }
    public Polyline3D GuidePath { get; }
    public Vector3D TopologyNormal { get; }
    public IReadOnlyDictionary<string, double>? Metadata { get; }
}

