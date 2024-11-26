
using System;
using BEPUutilities;

namespace BEPUphysics.EntityStateManagement
{
    ///<summary>
    /// State describing the position, orientation, and velocity of an entity.
    ///</summary>
    public struct MotionState : IEquatable<MotionState>
    {
        ///<summary>
        /// Position of an entity.
        ///</summary>
        public Vector3 Position;
        ///<summary>
        /// Orientation of an entity.
        ///</summary>
        public Quaternion Orientation;
        ///<summary>
        /// Orientation matrix of an entity.
        ///</summary>
        public Matrix OrientationMatrix
        {
            get
            {
                Matrix.CreateFromQuaternion(ref Orientation, out Matrix toReturn);
                return toReturn;
            }
        }
        ///<summary>
        /// World transform of an entity.
        ///</summary>
        public Matrix WorldTransform
        {
            get
            {
                Matrix.CreateFromQuaternion(ref Orientation, out Matrix toReturn);
                toReturn.Translation = Position;
                return toReturn;
            }
        }
        ///<summary>
        /// Linear velocity of an entity.
        ///</summary>
        public Vector3 LinearVelocity;
        ///<summary>
        /// Angular velocity of an entity.
        ///</summary>
        public Vector3 AngularVelocity;

        public readonly bool Equals(MotionState other)
            =>  other.AngularVelocity == AngularVelocity &&
                other.LinearVelocity == LinearVelocity &&
                other.Position == Position &&
                other.Orientation == Orientation;
    }
}
