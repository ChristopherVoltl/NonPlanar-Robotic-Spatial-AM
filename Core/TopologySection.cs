using System.Collections.Generic;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Describes one topology-derived section used as input to the nonplanar spatial slicer.
/// </summary>
/// <remarks>
/// A topology section is not the same thing as a final print layer. It is an intermediate guide extracted from the model that
/// the slicer can reorder, analyze, and orient. The class is immutable after construction and has no side-effects.
/// </remarks>
public sealed class TopologySection
{
    /// <summary>
    /// Initializes a topology section.
    /// </summary>
    /// <param name="index">A stable ordering key. Callers should avoid duplicates when deterministic ordering matters.</param>
    /// <param name="guidePath">The guide polyline for the section. It should not be <see langword="null"/>.</param>
    /// <param name="topologyNormal">The representative orientation for the section. Zero-length vectors are normalized through the vector fallback rules.</param>
    /// <param name="metadata">Optional scalar metadata for downstream heuristics or analysis.</param>
    /// <remarks>
    /// Preconditions: <paramref name="guidePath"/> should contain points in path order and usually at least two points for useful slicing.
    /// Postconditions: <paramref name="topologyNormal"/> is unitized before storage.
    /// Exceptions: this constructor does not throw by itself, but passing <see langword="null"/> for <paramref name="guidePath"/> will later cause callers to fail.
    /// Side-effects: none.
    /// </remarks>
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

