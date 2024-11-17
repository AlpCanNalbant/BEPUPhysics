using BEPUphysics.BroadPhaseEntries;
using BEPUutilities;

namespace BEPUphysics
{
    ///<summary>
    /// Contains information about a ray cast hit.
    ///</summary>
    ///<remarks>
    /// Constructs a new ray cast result.
    ///</remarks>
    ///<param name="hitData">Ray cast hit data.</param>
    ///<param name="hitObject">Object hit by the ray.</param>
    public struct RayCastResult(RayHit hitData, BroadPhaseEntry hitObject)
    {
        ///<summary>
        /// Position, normal, and t paramater of the hit.
        ///</summary>
        public RayHit HitData = hitData;
        /// <summary>
        /// Object hit by the ray.
        /// </summary>
        public BroadPhaseEntry HitObject = hitObject;
    }
}
