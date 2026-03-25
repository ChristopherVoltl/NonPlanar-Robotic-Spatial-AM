using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using SpatialAdditiveManufacturing.Core.Geometry;

namespace SpatialAdditiveManufacturing.Core.Slicing;

/// <summary>
/// Approximate S3-style slicer inspired by the paper's two-level structure:
/// an inner loop of quaternion-field optimization and an outer loop of
/// scale-controlled deformation that induces a scalar slicing field.
/// </summary>
public sealed class S3SlicerEngine
{
    public S3SliceFieldResult Generate(IReadOnlyList<S3NodeSample> nodes, S3SliceGenerationOptions options)
    {
        if (nodes is null)
        {
            throw new ArgumentNullException(nameof(nodes));
        }

        if (options is null)
        {
            throw new ArgumentNullException(nameof(options));
        }

        if (nodes.Count == 0)
        {
            return new S3SliceFieldResult(Array.Empty<Vector3D>(), Array.Empty<double>(), Array.Empty<double>(), Array.Empty<double>());
        }

        Vector3D up = options.GlobalUpAxis.Unitized();
        double layerHeight = Math.Max(options.LayerHeight, NumericalTolerance.Epsilon);
        var objectiveStrengths = new double[nodes.Count];
        var objectives = new Vector3D[nodes.Count];

        for (int i = 0; i < nodes.Count; i++)
        {
            objectives[i] = ComputeObjectiveDirection(nodes[i], options, up, out objectiveStrengths[i]);
        }

        var quaternions = objectives.Select(direction => QuaternionFromTo(up, direction)).ToArray();

        for (int iteration = 0; iteration < Math.Max(1, options.InnerIterations); iteration++)
        {
            var next = new Quaternion[quaternions.Length];
            for (int i = 0; i < nodes.Count; i++)
            {
                var candidates = new List<(Quaternion quat, double weight)>
                {
                    (QuaternionFromTo(up, objectives[i]), 1.0)
                };

                foreach (int neighborIndex in nodes[i].Neighbors)
                {
                    if (neighborIndex < 0 || neighborIndex >= quaternions.Length)
                    {
                        continue;
                    }

                    candidates.Add((quaternions[neighborIndex], options.QuaternionSmoothness));
                }

                next[i] = AverageQuaternions(candidates);
            }

            quaternions = next;
        }

        var optimizedDirections = quaternions
            .Select(quaternion => Rotate(quaternion, up).Unitized())
            .ToArray();
        var rawAngleWeights = optimizedDirections
            .Select(direction => Clamp01(direction.AngleTo(up) / 40.0))
            .ToArray();
        var smoothedAngleWeights = SmoothNodeWeights(rawAngleWeights, nodes, 2, 0.3);

        double minZ = nodes.Min(node => node.Position.Z);
        var baseScalars = nodes
            .Select(node => (node.Position.Z - minZ) / layerHeight)
            .ToArray();
        double maxBaseScalar = Math.Max(baseScalars.DefaultIfEmpty(0.0).Max(), NumericalTolerance.Epsilon);
        var layerProgress = baseScalars
            .Select(baseScalar => Math.Pow(Clamp01(baseScalar / maxBaseScalar), 0.7))
            .ToArray();
        var scalarValues = baseScalars.ToArray();
        var deformationScales = new double[nodes.Count];
        var printDirections = new Vector3D[nodes.Count];

        for (int outer = 0; outer < Math.Max(1, options.OuterIterations); outer++)
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                double fieldIntensity = ComputeFieldIntensity(options, objectiveStrengths[i], smoothedAngleWeights[i], layerProgress[i]);
                double targetScale = options.DeformationStrength
                    * fieldIntensity
                    * (1.35 + (1.05 * objectiveStrengths[i]));
                deformationScales[i] = Lerp(deformationScales[i], targetScale, 0.5);
            }

            for (int i = 0; i < nodes.Count; i++)
            {
                double fieldIntensity = ComputeFieldIntensity(options, objectiveStrengths[i], smoothedAngleWeights[i], layerProgress[i]);
                double directionBlend = Clamp01((0.15 * layerProgress[i]) + (fieldIntensity * (1.2 + (0.65 * deformationScales[i]))));
                printDirections[i] = BlendDirections(up, optimizedDirections[i], directionBlend);
            }

            var nextScalar = new double[scalarValues.Length];
            for (int i = 0; i < nodes.Count; i++)
            {
                IReadOnlyList<int> neighbors = nodes[i].Neighbors;
                if (neighbors.Count == 0)
                {
                    nextScalar[i] = scalarValues[i];
                    continue;
                }

                double accumulation = 0.0;
                double weightSum = 0.0;
                double fieldIntensity = ComputeFieldIntensity(options, objectiveStrengths[i], smoothedAngleWeights[i], layerProgress[i]);

                foreach (int neighborIndex in neighbors)
                {
                    if (neighborIndex < 0 || neighborIndex >= nodes.Count)
                    {
                        continue;
                    }

                    Vector3D edge = nodes[neighborIndex].Position - nodes[i].Position;
                    double edgeLength = Math.Max(edge.Length, NumericalTolerance.Epsilon);
                    Vector3D averageDirection = (printDirections[i] + printDirections[neighborIndex]).Unitized();
                    Vector3D lateralDirection = ProjectToPlane(averageDirection, up);
                    double lateralMagnitude = lateralDirection.Length;
                    if (lateralMagnitude > NumericalTolerance.Epsilon)
                    {
                        lateralDirection = lateralDirection / lateralMagnitude;
                    }

                    double verticalTransport = edge.Dot(averageDirection) / layerHeight;
                    double lateralTransport = edge.Dot(lateralDirection) / layerHeight;
                    double coupledDeformation = 0.5 * (deformationScales[i] + deformationScales[neighborIndex]);
                    double neighborIntensity = ComputeFieldIntensity(options, objectiveStrengths[neighborIndex], smoothedAngleWeights[neighborIndex], layerProgress[neighborIndex]);
                    double transportStrength = 0.5 * (fieldIntensity + neighborIntensity);
                    double desiredDifference =
                        verticalTransport +
                        (lateralTransport * (0.25 + (3.2 * transportStrength) + (1.1 * coupledDeformation)));
                    double weight = (1.0 / edgeLength) * (1.0 + (0.5 * transportStrength));

                    accumulation += (scalarValues[neighborIndex] - desiredDifference) * weight;
                    weightSum += weight;
                }

                double baseScalar = baseScalars[i];
                double anchorTarget = baseScalar;
                double anchorWeight = Lerp(2.25, 0.08, fieldIntensity);
                double compatibleScalar = weightSum <= NumericalTolerance.Epsilon
                    ? scalarValues[i]
                    : (accumulation + (anchorWeight * anchorTarget)) / (weightSum + anchorWeight);
                double deformationBias = deformationScales[i] * (0.35 + (1.85 * fieldIntensity));
                double relaxedTarget = compatibleScalar + deformationBias;
                nextScalar[i] = Lerp(scalarValues[i], relaxedTarget, 0.62 + (0.28 * fieldIntensity));
            }

            scalarValues = nextScalar;
        }

        var angles = printDirections
            .Select(direction => direction.AngleTo(up))
            .ToArray();

        return new S3SliceFieldResult(printDirections, scalarValues, deformationScales, angles);
    }

    private static Vector3D ComputeObjectiveDirection(S3NodeSample node, S3SliceGenerationOptions options, Vector3D up, out double objectiveStrength)
    {
        Vector3D support = AlignWithUp(EnsureTangent(node.SupportDirection, node.SurfaceNormal, up), up);
        Vector3D strength = AlignWithUp(EnsureTangent(node.CurvatureDirection, node.SurfaceNormal, support), up);
        Vector3D quality = BuildQualityDirection(node.SurfaceNormal, support, strength, up);
        double supportWeight = Math.Max(0.0, options.SupportFreeWeight);
        double strengthWeight = Math.Max(0.0, options.StrengthWeight);
        double qualityWeight = Math.Max(0.0, options.SurfaceQualityWeight);
        double totalWeight = supportWeight + strengthWeight + qualityWeight;
        if (totalWeight <= NumericalTolerance.Epsilon)
        {
            objectiveStrength = 0.0;
            return up;
        }

        Vector3D blended =
            (support * supportWeight) +
            (strength * strengthWeight) +
            (quality * qualityWeight);

        if (blended.Length <= NumericalTolerance.Epsilon)
        {
            objectiveStrength = 0.0;
            return up;
        }

        blended = AlignWithUp(blended, up);
        double pairwiseDivergence = (
            (1.0 - Math.Abs(support.Dot(strength))) +
            (1.0 - Math.Abs(support.Dot(quality))) +
            (1.0 - Math.Abs(strength.Dot(quality)))) / 3.0;
        double weightBias = Math.Max(supportWeight, Math.Max(strengthWeight, qualityWeight)) / totalWeight;
        objectiveStrength = Clamp01((0.55 * pairwiseDivergence) + (0.45 * weightBias));
        return blended.Unitized();
    }

    private static double ComputeFieldFollow(S3SliceGenerationOptions options, double objectiveStrength)
    {
        double deformation = Clamp01(options.DeformationStrength);
        return Clamp01((0.2 + (0.8 * deformation)) * (0.65 + (0.7 * objectiveStrength)));
    }

    private static double ComputeFieldIntensity(S3SliceGenerationOptions options, double objectiveStrength, double angleWeight, double layerProgress)
    {
        double fieldFollow = ComputeFieldFollow(options, objectiveStrength);
        double weightedAngle = Math.Pow(Clamp01(angleWeight), 0.6);
        double weightedProgress = 0.15 + (0.85 * Math.Pow(Clamp01(layerProgress), 0.75));
        return Clamp01((0.25 + (0.75 * fieldFollow)) * weightedAngle * weightedProgress);
    }

    private static Vector3D BlendDirections(Vector3D start, Vector3D end, double t)
    {
        return Vector3D.Lerp(start, end, Clamp01(t));
    }

    private static double[] SmoothNodeWeights(IReadOnlyList<double> values, IReadOnlyList<S3NodeSample> nodes, int iterations, double neighborWeight)
    {
        var current = values.ToArray();
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            var next = new double[current.Length];
            for (int i = 0; i < current.Length; i++)
            {
                double sum = current[i];
                double weightSum = 1.0;
                IReadOnlyList<int> neighbors = nodes[i].Neighbors;
                for (int j = 0; j < neighbors.Count; j++)
                {
                    int neighborIndex = neighbors[j];
                    if (neighborIndex < 0 || neighborIndex >= current.Length)
                    {
                        continue;
                    }

                    sum += current[neighborIndex] * neighborWeight;
                    weightSum += neighborWeight;
                }

                next[i] = weightSum <= NumericalTolerance.Epsilon ? current[i] : (sum / weightSum);
            }

            current = next;
        }

        return current;
    }

    private static Vector3D EnsureTangent(Vector3D direction, Vector3D surfaceNormal, Vector3D fallback)
    {
        Vector3D tangent = ProjectToPlane(direction, surfaceNormal);
        if (tangent.Length <= NumericalTolerance.Epsilon)
        {
            tangent = ProjectToPlane(fallback, surfaceNormal);
        }

        return tangent.Length <= NumericalTolerance.Epsilon ? fallback : tangent.Unitized();
    }

    private static Vector3D BuildQualityDirection(Vector3D surfaceNormal, Vector3D support, Vector3D strength, Vector3D up)
    {
        Vector3D quality = surfaceNormal.Cross(support);
        if (quality.Length <= NumericalTolerance.Epsilon)
        {
            quality = surfaceNormal.Cross(strength);
        }

        quality = ProjectToPlane(quality, surfaceNormal);
        if (quality.Length <= NumericalTolerance.Epsilon)
        {
            quality = ProjectToPlane(up, surfaceNormal);
        }

        return AlignWithUp(quality, up);
    }

    private static Vector3D ProjectToPlane(Vector3D vector, Vector3D planeNormal)
    {
        Vector3D unitNormal = planeNormal.Unitized();
        return vector - (unitNormal * vector.Dot(unitNormal));
    }

    private static Vector3D AlignWithUp(Vector3D vector, Vector3D up)
    {
        if (vector.Length <= NumericalTolerance.Epsilon)
        {
            return up;
        }

        Vector3D unit = vector.Unitized();
        return unit.Dot(up) < 0.0 ? unit * -1.0 : unit;
    }

    private static Quaternion QuaternionFromTo(Vector3D from, Vector3D to)
    {
        Vector3 v0 = ToNumerics(from.Unitized());
        Vector3 v1 = ToNumerics(to.Unitized());
        float dot = Vector3.Dot(v0, v1);

        if (dot > 0.9999f)
        {
            return Quaternion.Identity;
        }

        if (dot < -0.9999f)
        {
            Vector3 axis = Vector3.Cross(v0, Vector3.UnitX);
            if (axis.LengthSquared() < 1e-8f)
            {
                axis = Vector3.Cross(v0, Vector3.UnitY);
            }

            axis = Vector3.Normalize(axis);
            return Quaternion.CreateFromAxisAngle(axis, (float)Math.PI);
        }

        Vector3 cross = Vector3.Cross(v0, v1);
        Quaternion quaternion = new(cross, 1.0f + dot);
        quaternion = Quaternion.Normalize(quaternion);
        return quaternion;
    }

    private static Quaternion AverageQuaternions(IReadOnlyList<(Quaternion quat, double weight)> quaternions)
    {
        if (quaternions.Count == 0)
        {
            return Quaternion.Identity;
        }

        Quaternion seed = quaternions[0].quat;
        Vector4 accumulation = Vector4.Zero;

        foreach ((Quaternion quat, double weight) in quaternions)
        {
            Quaternion candidate = quat;
            if (Quaternion.Dot(seed, candidate) < 0.0f)
            {
                candidate = new Quaternion(-candidate.X, -candidate.Y, -candidate.Z, -candidate.W);
            }

            accumulation += new Vector4(candidate.X, candidate.Y, candidate.Z, candidate.W) * (float)weight;
        }

        Quaternion averaged = new(accumulation.X, accumulation.Y, accumulation.Z, accumulation.W);
        if (averaged.LengthSquared() <= 1e-8f)
        {
            return seed;
        }

        return Quaternion.Normalize(averaged);
    }

    private static Vector3D Rotate(Quaternion quaternion, Vector3D vector)
    {
        Vector3 rotated = Vector3.Transform(ToNumerics(vector), quaternion);
        return new Vector3D(rotated.X, rotated.Y, rotated.Z);
    }

    private static Vector3 ToNumerics(Vector3D vector) => new((float)vector.X, (float)vector.Y, (float)vector.Z);

    private static double Lerp(double start, double end, double t)
    {
        double clamped = Math.Max(0.0, Math.Min(1.0, t));
        return start + ((end - start) * clamped);
    }

    private static double Clamp(double value, double min, double max)
    {
        return Math.Max(min, Math.Min(max, value));
    }

    private static double Clamp01(double value)
    {
        return Clamp(value, 0.0, 1.0);
    }

}
