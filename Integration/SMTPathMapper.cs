using Rhino.Geometry;

namespace NonPlanar_Robotic_Spatial_AM
{
    /// <summary>
    /// Shared orientation and parameter mapping layer used by SMT export.
    /// Keep all E5 and velocity classification rules here so different slicers
    /// can reuse the same SMT translation behavior.
    /// </summary>
    public static class SMTPathMapper
    {
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

    public enum SegmentOrientationStyle
    {
        Vertical,
        Angled,
        Horizontal
    }
}

