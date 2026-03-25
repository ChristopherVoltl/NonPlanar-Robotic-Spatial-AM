using System.Collections.Generic;

namespace SpatialAdditiveManufacturing.Core.Slicing;

public interface ITopologySectionProvider<in TGeometry>
{
    IReadOnlyList<TopologySection> ExtractSections(TGeometry geometry);
}

