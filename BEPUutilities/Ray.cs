using System;

namespace BEPUutilities
{
    /// <summary>
    /// Provides XNA-like ray functionality.
    /// </summary>
    /// <remarks>
    /// Constructs a new ray.
    /// </remarks>
    /// <param name="position">Starting position of the ray.</param>
    /// <param name="direction">Direction in which the ray points.</param>
    public struct Ray(Vector3 position, Vector3 direction)
    {
        /// <summary>
        /// Starting position of the ray.
        /// </summary>
        public Vector3 Position = position;
        /// <summary>
        /// Direction in which the ray points.
        /// </summary>
        public Vector3 Direction = direction;



        /// <summary>
        /// Determines if and when the ray intersects the bounding box.
        /// </summary>
        /// <param name="boundingBox">Bounding box to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        public readonly bool Intersects(ref BoundingBox boundingBox, out float t)
        {
            float tmin = 0, tmax = float.MaxValue;
            if (Math.Abs(Direction.X) < Toolbox.Epsilon)
            {
                if (Position.X < boundingBox.Min.X || Position.X > boundingBox.Max.X)
                {
                    //If the ray isn't pointing along the axis at all, and is outside of the box's interval, then it
                    //can't be intersecting.
                    t = 0;
                    return false;
                }
            }
            else
            {
                var inverseDirection = 1 / Direction.X;
                var t1 = (boundingBox.Min.X - Position.X) * inverseDirection;
                var t2 = (boundingBox.Max.X - Position.X) * inverseDirection;
                if (t1 > t2)
                {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                }
                tmin = Math.Max(tmin, t1);
                tmax = Math.Min(tmax, t2);
                if (tmin > tmax)
                {
                    t = 0;
                    return false;
                }
            }
            if (Math.Abs(Direction.Y) < Toolbox.Epsilon)
            {
                if (Position.Y < boundingBox.Min.Y || Position.Y > boundingBox.Max.Y)
                {
                    //If the ray isn't pointing along the axis at all, and is outside of the box's interval, then it
                    //can't be intersecting.
                    t = 0;
                    return false;
                }
            }
            else
            {
                var inverseDirection = 1 / Direction.Y;
                var t1 = (boundingBox.Min.Y - Position.Y) * inverseDirection;
                var t2 = (boundingBox.Max.Y - Position.Y) * inverseDirection;
                if (t1 > t2)
                {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                }
                tmin = Math.Max(tmin, t1);
                tmax = Math.Min(tmax, t2);
                if (tmin > tmax)
                {
                    t = 0;
                    return false;
                }
            }
            if (Math.Abs(Direction.Z) < Toolbox.Epsilon)
            {
                if (Position.Z < boundingBox.Min.Z || Position.Z > boundingBox.Max.Z)
                {
                    //If the ray isn't pointing along the axis at all, and is outside of the box's interval, then it
                    //can't be intersecting.
                    t = 0;
                    return false;
                }
            }
            else
            {
                var inverseDirection = 1 / Direction.Z;
                var t1 = (boundingBox.Min.Z - Position.Z) * inverseDirection;
                var t2 = (boundingBox.Max.Z - Position.Z) * inverseDirection;
                if (t1 > t2)
                {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                }
                tmin = Math.Max(tmin, t1);
                tmax = Math.Min(tmax, t2);
                if (tmin > tmax)
                {
                    t = 0;
                    return false;
                }
            }
            t = tmin;
            return true;
        }

        /// <summary>
        /// Determines if and when the ray intersects the bounding box.
        /// </summary>
        /// <param name="boundingBox">Bounding box to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        public readonly bool Intersects(BoundingBox boundingBox, out float t)
            => Intersects(ref boundingBox, out t);

        /// <summary>
        /// Determines if and when the ray intersects the plane.
        /// </summary>
        /// <param name="plane">Plane to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        public bool Intersects(ref Plane plane, out float t)
        {
            Vector3.Dot(ref Direction, ref plane.Normal, out float velocity);
            if (Math.Abs(velocity) < Toolbox.Epsilon)
            {
                t = 0;
                return false;
            }
            Vector3.Dot(ref Position, ref plane.Normal, out float distanceAlongNormal);
            distanceAlongNormal += plane.D;
            t = -distanceAlongNormal / velocity;
            return t >= -Toolbox.Epsilon;
        }

        /// <summary>
        /// Determines if and when the ray intersects the plane.
        /// </summary>
        /// <param name="plane">Plane to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        public bool Intersects(Plane plane, out float t)
            => Intersects(ref plane, out t);

        /// <summary>
        /// Computes a point along a ray given the length along the ray from the ray position.
        /// </summary>
        /// <param name="t">Length along the ray from the ray position in terms of the ray's direction.</param>
        /// <param name="v">Point along the ray at the given location.</param>
        public void GetPointOnRay(float t, out Vector3 v)
        {
            Vector3.Multiply(ref Direction, t, out v);
            Vector3.Add(ref v, ref Position, out v);
        }
    }
}
