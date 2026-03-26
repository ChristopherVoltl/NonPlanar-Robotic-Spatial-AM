using Rhino.Geometry;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Shared orientation and parameter mapping layer used by SMT export.
    /// Keep all E5 and velocity classification rules here so different slicers
    /// can reuse the same SMT translation behavior.
    /// </summary>
    /// <remarks>
    /// This class exists to keep machine-specific mapping rules out of the geometric slicers. It contains no global state
    /// and all methods are deterministic for a given input segment.
    /// </remarks>
    public static class SMTPathMapper
    {
        /// <summary>
        /// Classifies a segment as vertical, angled, or horizontal relative to global Z.
        /// </summary>
        /// <param name="line">The segment to classify.</param>
        /// <returns>
        /// A <see cref="SegmentOrientationStyle"/> value. Degenerate segments fall back to <see cref="SegmentOrientationStyle.Horizontal"/>.
        /// </returns>
        /// <remarks>
        /// Preconditions: callers should provide line geometry in the same global coordinate system used for printing.
        /// Postconditions: the result can be used consistently with <see cref="GetE5Value"/> and <see cref="GetVelocityRatio"/>.
        /// Exceptions: none.
        /// Side-effects: none.
        /// </remarks>
        public static SegmentOrientationStyle Classify(Line line)
        {
            Vector3d direction = line.Direction;
            if (!direction.Unitize())
            {
                return SegmentOrientationStyle.Horizontal;
            }

            double angleToGlobalZ = Vector3d.VectorAngle(direction, Vector3d.ZAxis) * (180.0 / System.Math.PI);

            if (angleToGlobalZ <= 20.0 || angleToGlobalZ >= 160.0)
            {
                return SegmentOrientationStyle.Vertical;
            }

            if (angleToGlobalZ >= 70.0 && angleToGlobalZ <= 110.0)
            {
                return SegmentOrientationStyle.Horizontal;
            }

            return SegmentOrientationStyle.Angled;
        }

        /// <summary>
        /// Maps an orientation classification to the configured E5 value.
        /// </summary>
        /// <remarks>
        /// Preconditions: callers should pass the same style produced by <see cref="Classify"/> for the segment being exported.
        /// Postconditions: one of the supplied E5 values is returned unchanged.
        /// Exceptions: none.
        /// Differences: unlike <see cref="GetVelocityRatio"/>, this method does not apply any multiplier or internal defaults beyond the style switch.
        /// Side-effects: none.
        /// </remarks>
        public static double GetE5Value(
            SegmentOrientationStyle style,
            double verticalE5,
            double angledE5,
            double horizontalE5)
        {
            switch (style)
            {
                case SegmentOrientationStyle.Vertical:
                    return verticalE5;
                case SegmentOrientationStyle.Angled:
                    return angledE5;
                default:
                    return horizontalE5;
            }
        }

        /// <summary>
        /// Maps an orientation classification to a velocity ratio for SMT.
        /// </summary>
        /// <param name="style">The orientation style of the current segment.</param>
        /// <param name="velocityRatioMultiplier">A caller-defined multiplier applied to the style's base ratio.</param>
        /// <returns>A velocity ratio as a <see cref="float"/> for SMT point data.</returns>
        /// <remarks>
        /// Preconditions: callers should supply a positive multiplier for predictable output.
        /// Postconditions: the returned value includes both the style-specific base ratio and the multiplier.
        /// Exceptions: none.
        /// Side-effects: none.
        /// </remarks>
        public static float GetVelocityRatio(SegmentOrientationStyle style, double velocityRatioMultiplier)
        {
            double baseRatio;
            switch (style)
            {
                case SegmentOrientationStyle.Vertical:
                    baseRatio = 0.8;
                    break;
                case SegmentOrientationStyle.Angled:
                    baseRatio = 1.0;
                    break;
                default:
                    baseRatio = 1.15;
                    break;
            }

            return (float)(baseRatio * velocityRatioMultiplier);
        }

        /// <summary>
        /// Creates an SMT path plane from a line segment.
        /// </summary>
        /// <param name="line">The source segment. Degenerate segments fall back to the global X axis.</param>
        /// <returns>
        /// A plane whose origin is the segment start and whose X axis follows the segment direction.
        /// </returns>
        /// <remarks>
        /// This method exists because SMT point data expects an orientation plane per path point or segment.
        /// Preconditions: callers should provide lines in global model coordinates.
        /// Postconditions: the returned plane is always constructed, using fallback axes when needed.
        /// Exceptions: none.
        /// Side-effects: none.
        /// </remarks>
        public static Plane CreatePathPlane(Line line)
        {
            Point3d origin = line.From;
            Vector3d xAxis = line.Direction;
            if (!xAxis.Unitize())
            {
                xAxis = Vector3d.XAxis;
            }

            Vector3d referenceUp = Vector3d.ZAxis;
            Vector3d yAxis = Vector3d.CrossProduct(referenceUp, xAxis);
            if (!yAxis.Unitize())
            {
                yAxis = Vector3d.YAxis;
            }

            return new Plane(origin, xAxis, yAxis);
        }
    }

    /// <summary>
    /// The coarse orientation buckets used when mapping path segments to SMT process settings.
    /// </summary>
    public enum SegmentOrientationStyle
    {
        Vertical,
        Angled,
        Horizontal
    }
}

