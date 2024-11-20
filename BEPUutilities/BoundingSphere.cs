using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUutilities
{
    /// <summary>
    /// Provides XNA-like bounding sphere functionality.
    /// </summary>
    /// <remarks>
    /// Constructs a new bounding sphere.
    /// </remarks>
    /// <param name="center">Location of the center of the sphere.</param>
    /// <param name="radius">Radius of the sphere.</param>
    public struct BoundingSphere(Vector3 center, float radius)
    {
        /// <summary>
        /// Radius of the sphere.
        /// </summary>
        public float Radius = radius;
        /// <summary>
        /// Location of the center of the sphere.
        /// </summary>
        public Vector3 Center = center;
    }
}
