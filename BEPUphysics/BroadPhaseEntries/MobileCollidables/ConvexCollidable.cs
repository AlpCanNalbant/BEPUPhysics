using BEPUphysics.BroadPhaseEntries.Events;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUutilities;

namespace BEPUphysics.BroadPhaseEntries.MobileCollidables
{
    ///<summary>
    /// Collidable with a convex shape.
    ///</summary>
    public abstract class ConvexCollidable : EntityCollidable
    {

        protected ConvexCollidable(ConvexShape shape)
            : base(shape)
            => Events = new ContactEventManager<EntityCollidable>();

        ///<summary>
        /// Gets the shape of the collidable.
        ///</summary>
        public new ConvexShape Shape
                => (ConvexShape)shape;


        public override bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, out RayHit hit)
            => MPRToolbox.Sweep(castShape, Shape, ref sweep, ref Toolbox.ZeroVector, ref startingTransform, ref worldTransform, out hit);

    }

    ///<summary>
    /// Collidable with a convex shape of a particular type.
    ///</summary>
    ///<typeparam name="T">ConvexShape type.</typeparam>
    ///<remarks>
    /// Constructs a new convex collidable.
    ///</remarks>
    ///<param name="shape">Shape to use in the collidable.</param>
    public class ConvexCollidable<T>(T shape) : ConvexCollidable(shape) where T : ConvexShape
    {
        ///<summary>
        /// Gets the shape of the collidable.
        ///</summary>
        public new T Shape
                => (T)shape;


        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public override bool RayCast(Ray ray, float maximumLength, out RayHit rayHit)
            => Shape.RayTest(ref ray, ref worldTransform, maximumLength, out rayHit);



        protected internal override void UpdateBoundingBoxInternal(float dt)
        {
            Shape.GetBoundingBox(ref worldTransform, out boundingBox);

            ExpandBoundingBox(ref boundingBox, dt);
        }

        // (WCS Edit) New method added.
        public float CalculateBoundingBoxLength()
            => Shape.GetBoundingBox(ref worldTransform).CalculateLength();



    }
}
