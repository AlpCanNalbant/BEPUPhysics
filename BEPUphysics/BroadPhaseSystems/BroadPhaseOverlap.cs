using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.CollisionRuleManagement;

namespace BEPUphysics.BroadPhaseSystems
{
    /// <summary>
    /// A pair of overlapping BroadPhaseEntries.
    /// </summary>
    public struct BroadPhaseOverlap : IEquatable<BroadPhaseOverlap>
    {
        internal BroadPhaseEntry entryA;
        /// <summary>
        /// First entry in the pair.
        /// </summary>
        public readonly BroadPhaseEntry EntryA
            => entryA;

        internal BroadPhaseEntry entryB;
        /// <summary>
        /// Second entry in the pair.
        /// </summary>
        public readonly BroadPhaseEntry EntryB
            => entryB;

        internal CollisionRule collisionRule;

        /// <summary>
        /// Constructs an overlap.
        /// </summary>
        /// <param name="entryA">First entry in the pair.</param>
        /// <param name="entryB">Second entry in the pair.</param>
        public BroadPhaseOverlap(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            this.entryA = entryA;
            this.entryB = entryB;
            collisionRule = CollisionRules.DefaultCollisionRule;
        }

        /// <summary>
        /// Constructs an overlap.
        /// </summary>
        /// <param name="entryA">First entry in the pair.</param>
        /// <param name="entryB">Second entry in the pair.</param>
        /// <param name="collisionRule">Collision rule calculated for the pair.</param>
        public BroadPhaseOverlap(BroadPhaseEntry entryA, BroadPhaseEntry entryB, CollisionRule collisionRule)
        {
            this.entryA = entryA;
            this.entryB = entryB;
            this.collisionRule = collisionRule;
        }

        /// <summary>
        /// Gets the collision rule calculated for the pair.
        /// </summary>
        public readonly CollisionRule CollisionRule
            => collisionRule;

        /// <summary>
        /// Gets the hash code of the object.
        /// </summary>
        /// <returns>Hash code of the object.</returns>
        public override readonly int GetHashCode()
        // {
            //TODO: Use old prime-based system?
            => (int)((entryA.hashCode + entryB.hashCode) * 0xd8163841);
        // }


        #region IEquatable<BroadPhaseOverlap> Members

        /// <summary>
        /// Compares the overlaps for equality based on the involved entries.
        /// </summary>
        /// <param name="other">Overlap to compare.</param>
        /// <returns>Whether or not the overlaps were equal.</returns>
        public readonly bool Equals(BroadPhaseOverlap other)
            => (other.entryA == entryA && other.entryB == entryB) || (other.entryA == entryB && other.entryB == entryA);

        #endregion

        public override readonly string ToString()
            => "{" + entryA + ", " + entryB + "}";
    }
}
