using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.BroadPhaseSystems.Hierarchies;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Constraints;
using BEPUphysics.DeactivationManagement;
using BEPUphysics.Entities;
using BEPUphysics.EntityStateManagement;
using BEPUphysics.OtherSpaceStages;
using BEPUphysics.PositionUpdating;
using BEPUutilities;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.UpdateableSystems;
using BEPUutilities.DataStructures;
using BEPUutilities.Threading;

namespace BEPUphysics
{
    ///<summary>
    /// Main simulation class of BEPUphysics.  Contains various updating stages addition/removal methods for getting objects into the simulation.
    ///</summary>
    public class Space
    {
        private TimeStepSettings timeStepSettings;
        ///<summary>
        /// Gets or sets the time step settings used by the space.
        ///</summary>
        public TimeStepSettings TimeStepSettings
        {
            get => timeStepSettings;
            set
            {
                timeStepSettings = value;
                DeactivationManager.TimeStepSettings = value;
                ForceUpdater.TimeStepSettings = value;
                BoundingBoxUpdater.TimeStepSettings = value;
                Solver.TimeStepSettings = value;
                PositionUpdater.TimeStepSettings = value;
            }
        }

        IParallelLooper parallelLooper;
        ///<summary>
        /// Gets or sets the parallel loop provider used by the space.
        ///</summary>
        public IParallelLooper ParallelLooper
        {
            get => parallelLooper;
            set
            {
                parallelLooper = value;
                DeactivationManager.ParallelLooper = value;
                ForceUpdater.ParallelLooper = value;
                BoundingBoxUpdater.ParallelLooper = value;
                BroadPhase.ParallelLooper = value;
                NarrowPhase.ParallelLooper = value;
                Solver.ParallelLooper = value;
                PositionUpdater.ParallelLooper = value;
                DuringForcesUpdateables.ParallelLooper = value;
                BeforeNarrowPhaseUpdateables.ParallelLooper = value;
                EndOfTimeStepUpdateables.ParallelLooper = value;
                EndOfFrameUpdateables.ParallelLooper = value;
            }
        }

        ///<summary>
        /// Gets or sets the space object buffer used by the space.
        /// The space object buffer allows objects to be safely asynchronously
        /// added to and removed from the space.
        ///</summary>
        public SpaceObjectBuffer SpaceObjectBuffer { get; set; }
        ///<summary>
        /// Gets or sets the entity state write buffer used by the space.
        /// The write buffer contains buffered writes to entity states that are
        /// flushed each frame when the buffer is updated.
        ///</summary>
        public EntityStateWriteBuffer EntityStateWriteBuffer { get; set; }
        ///<summary>
        /// Gets or sets the deactivation manager used by the space.
        /// The deactivation manager controls the activity state objects, putting them
        /// to sleep and managing the connections between objects and simulation islands.
        ///</summary>
        public DeactivationManager DeactivationManager { get; set; }
        ///<summary>
        /// Gets or sets the force updater used by the space.
        /// The force updater applies forces to all dynamic objects in the space each frame.
        ///</summary>
        public ForceUpdater ForceUpdater { get; set; }
        ///<summary>
        /// Gets or sets the bounding box updater used by the space.
        /// The bounding box updater updates the bounding box of mobile collidables each frame.
        ///</summary>
        public BoundingBoxUpdater BoundingBoxUpdater { get; set; }
        private BroadPhase broadPhase;
        /// <summary>
        /// Gets or sets the broad phase used by the space.
        /// The broad phase finds overlaps between broad phase entries and passes
        /// them off to the narrow phase for processing.
        /// </summary>
        public BroadPhase BroadPhase
        {
            get => broadPhase;
            set
            {
                broadPhase = value;
                if (NarrowPhase != null)
                {
                    if (value != null)
                    {
                        NarrowPhase.BroadPhaseOverlaps = broadPhase.Overlaps;
                    }
                    else
                    {
                        NarrowPhase.BroadPhaseOverlaps = null;
                    }
                }
            }
        }
        ///<summary>
        /// Gets or sets the narrow phase used by the space.
        /// The narrow phase uses overlaps found by the broad phase
        /// to create pair handlers.  Those pair handlers can go on to
        /// create things like contacts and constraints.
        ///</summary>
        public NarrowPhase NarrowPhase { get; set; }
        ///<summary>
        /// Gets or sets the solver used by the space.
        /// The solver iteratively finds a solution to the constraints in the simulation.
        ///</summary>
        public Solver Solver { get; set; }
        ///<summary>
        /// Gets or sets the position updater used by the space.
        /// The position updater moves everything around each frame.
        ///</summary>
        public PositionUpdater PositionUpdater { get; set; }
        ///<summary>
        /// Gets or sets the buffered states manager used by the space.
        /// The buffered states manager keeps track of read buffered entity states
        /// and also interpolated states based on the time remaining from internal
        /// time steps.
        ///</summary>
        public BufferedStatesManager BufferedStates { get; set; }
        ///<summary>
        /// Gets or sets the deferred event dispatcher used by the space.
        /// The event dispatcher gathers up deferred events created
        /// over the course of a timestep and dispatches them sequentially at the end.
        ///</summary>
        public DeferredEventDispatcher DeferredEventDispatcher { get; set; }

        ///<summary>
        /// Gets or sets the updateable manager that handles updateables that update during force application.
        ///</summary>
        public DuringForcesUpdateableManager DuringForcesUpdateables { get; set; }
        ///<summary>
        /// Gets or sets the updateable manager that handles updateables that update before the narrow phase.
        ///</summary>
        public BeforeNarrowPhaseUpdateableManager BeforeNarrowPhaseUpdateables { get; set; }
        ///<summary>
        /// Gets or sets the updateable manager that handles updateables that update before the solver
        ///</summary>
        public BeforeSolverUpdateableManager BeforeSolverUpdateables { get; set; }
        ///<summary>
        /// Gets or sets the updateable manager that handles updateables that update right before the position update phase.
        ///</summary>
        public BeforePositionUpdateUpdateableManager BeforePositionUpdateUpdateables { get; set; }
        ///<summary>
        /// Gets or sets the updateable manager that handles updateables that update at the end of a timestep.
        ///</summary>
        public EndOfTimeStepUpdateableManager EndOfTimeStepUpdateables { get; set; }
        ///<summary>
        /// Gets or sets the updateable manager that handles updateables that update at the end of a frame.
        ///</summary>
        public EndOfFrameUpdateableManager EndOfFrameUpdateables { get; set; }


        ///<summary>
        /// Gets the list of entities in the space.
        ///</summary>
        public ReadOnlyList<Entity> Entities
            => BufferedStates.Entities;

        ///<summary>
        /// Constructs a new space for things to live in.
        /// This overload does not provide an IParallelLooper, so it makes the space single threaded. Use the other overload or set the Space.ParallelLooper property to use multiple threads.
        ///</summary>
        public Space()
            : this(null)
        { }

        ///<summary>
        /// Constructs a new space for things to live in.
        ///</summary>
        ///<param name="parallelLooper">Used by the space to perform multithreaded updates. Pass null if multithreading is not required.</param>
        public Space(IParallelLooper parallelLooper)
        {
            timeStepSettings = new TimeStepSettings();

            this.parallelLooper = parallelLooper;

            SpaceObjectBuffer = new SpaceObjectBuffer(this);
            EntityStateWriteBuffer = new EntityStateWriteBuffer();
            DeactivationManager = new DeactivationManager(TimeStepSettings, ParallelLooper);
            ForceUpdater = new ForceUpdater(TimeStepSettings, ParallelLooper);
            BoundingBoxUpdater = new BoundingBoxUpdater(TimeStepSettings, ParallelLooper);
            BroadPhase = new DynamicHierarchy(ParallelLooper);
            NarrowPhase = new NarrowPhase(TimeStepSettings, BroadPhase.Overlaps, ParallelLooper);
            Solver = new Solver(TimeStepSettings, DeactivationManager, ParallelLooper);
            NarrowPhase.Solver = Solver;
            PositionUpdater = new ContinuousPositionUpdater(TimeStepSettings, ParallelLooper);
            BufferedStates = new BufferedStatesManager(ParallelLooper);
            DeferredEventDispatcher = new DeferredEventDispatcher();

            DuringForcesUpdateables = new DuringForcesUpdateableManager(timeStepSettings, ParallelLooper);
            BeforeNarrowPhaseUpdateables = new BeforeNarrowPhaseUpdateableManager(timeStepSettings, ParallelLooper);
            BeforeSolverUpdateables = new BeforeSolverUpdateableManager(timeStepSettings, ParallelLooper);
            BeforePositionUpdateUpdateables = new BeforePositionUpdateUpdateableManager(timeStepSettings, ParallelLooper);
            EndOfTimeStepUpdateables = new EndOfTimeStepUpdateableManager(timeStepSettings, ParallelLooper);
            EndOfFrameUpdateables = new EndOfFrameUpdateableManager(timeStepSettings, ParallelLooper);
        }

        ///<summary>
        /// Adds a space object to the simulation.
        ///</summary>
        ///<param name="spaceObject">Space object to add.</param>
        public bool Add(ISpaceObject spaceObject)
        {
            if (spaceObject.Space == this)
            {
                //
                // throw new ArgumentException("The object belongs to some Space already; cannot add it again.");
                // Do not throw WCS edit. Try remove first and then if cannot removed return false.
                if (!Remove(spaceObject))
                {
                    return false;
                }

                // If it's removed from the space, flow-through/continue to below lines ...
            }

            spaceObject.Space = this;

            if (spaceObject is SimulationIslandMember simulationIslandMember)
            {
                DeactivationManager.Add(simulationIslandMember);
            }

            if (spaceObject is ISimulationIslandMemberOwner simulationIslandMemberOwner)
            {
                DeactivationManager.Add(simulationIslandMemberOwner.ActivityInformation);
            }

            //Go through each stage, adding the space object to it if necessary.
            if (spaceObject is IForceUpdateable velocityUpdateable)
            {
                ForceUpdater.Add(velocityUpdateable);
            }

            if (spaceObject is MobileCollidable boundingBoxUpdateable)
            {
                BoundingBoxUpdater.Add(boundingBoxUpdateable);
            }

            if (spaceObject is BroadPhaseEntry broadPhaseEntry)
            {
                BroadPhase.Add(broadPhaseEntry);
            }

            //Entites own collision proxies, but are not entries themselves.
            if (spaceObject is IBroadPhaseEntryOwner broadPhaseEntryOwner)
            {
                BroadPhase.Add(broadPhaseEntryOwner.Entry);
                boundingBoxUpdateable = broadPhaseEntryOwner.Entry as MobileCollidable;
                if (boundingBoxUpdateable != null)
                {
                    BoundingBoxUpdater.Add(boundingBoxUpdateable);
                }
            }

            if (spaceObject is SolverUpdateable solverUpdateable)
            {
                Solver.Add(solverUpdateable);
            }

            if (spaceObject is IPositionUpdateable integrable)
            {
                PositionUpdater.Add(integrable);
            }

            if (spaceObject is Entity entity)
            {
                BufferedStates.Add(entity);
            }

            if (spaceObject is IDeferredEventCreator deferredEventCreator)
            {
                DeferredEventDispatcher.AddEventCreator(deferredEventCreator);
            }

            if (spaceObject is IDeferredEventCreatorOwner deferredEventCreatorOwner)
            {
                DeferredEventDispatcher.AddEventCreator(deferredEventCreatorOwner.EventCreator);
            }

            //Updateable stages.
            if (spaceObject is IDuringForcesUpdateable duringForcesUpdateable)
            {
                DuringForcesUpdateables.Add(duringForcesUpdateable);
            }

            if (spaceObject is IBeforeNarrowPhaseUpdateable beforeNarrowPhaseUpdateable)
            {
                BeforeNarrowPhaseUpdateables.Add(beforeNarrowPhaseUpdateable);
            }

            if (spaceObject is IBeforeSolverUpdateable beforeSolverUpdateable)
            {
                BeforeSolverUpdateables.Add(beforeSolverUpdateable);
            }

            if (spaceObject is IBeforePositionUpdateUpdateable beforePositionUpdateUpdateable)
            {
                BeforePositionUpdateUpdateables.Add(beforePositionUpdateUpdateable);
            }

            if (spaceObject is IEndOfTimeStepUpdateable endOfStepUpdateable)
            {
                EndOfTimeStepUpdateables.Add(endOfStepUpdateable);
            }

            if (spaceObject is IEndOfFrameUpdateable endOfFrameUpdateable)
            {
                EndOfFrameUpdateables.Add(endOfFrameUpdateable);
            }

            spaceObject.OnAdditionToSpace(this);
            return true;

            /*
            SimulationIslandMember simulationIslandMember = null;
            ISimulationIslandMemberOwner simulationIslandMemberOwner = null;
            IForceUpdateable velocityUpdateable = null;
            MobileCollidable boundingBoxUpdateable = null;
            BroadPhaseEntry broadPhaseEntry = null;
            IBroadPhaseEntryOwner broadPhaseEntryOwner = null;
            MobileCollidable broadPhaseBoundingBoxUpdateable = null;
            SolverUpdateable solverUpdateable = null;
            IPositionUpdateable integrable = null;
            Entity entity = null;
            IDeferredEventCreator deferredEventCreator = null;
            IDeferredEventCreatorOwner deferredEventCreatorOwner = null;
            IDuringForcesUpdateable duringForcesUpdateable = null;
            IBeforeNarrowPhaseUpdateable beforeNarrowPhaseUpdateable = null;
            IBeforeSolverUpdateable beforeSolverUpdateable = null;
            IBeforePositionUpdateUpdateable beforePositionUpdateUpdateable = null;
            IEndOfTimeStepUpdateable endOfStepUpdateable = null;
            IEndOfFrameUpdateable endOfFrameUpdateable = null;
            try
            {
                if ((simulationIslandMember = (SimulationIslandMember)spaceObject) != null)
                {
                    DeactivationManager.Add(simulationIslandMember);
                }

                if ((simulationIslandMemberOwner = (ISimulationIslandMemberOwner)spaceObject) != null)
                {
                    DeactivationManager.Add(simulationIslandMemberOwner.ActivityInformation);
                }

                //Go through each stage, adding the space object to it if necessary.
                if ((velocityUpdateable = (IForceUpdateable)spaceObject) != null)
                {
                    ForceUpdater.Add(velocityUpdateable);
                }

                if ((boundingBoxUpdateable = (MobileCollidable)spaceObject) != null)
                {
                    BoundingBoxUpdater.Add(boundingBoxUpdateable);
                }

                if ((broadPhaseEntry = (BroadPhaseEntry)spaceObject) != null)
                {
                    BroadPhase.Add(broadPhaseEntry);
                }

                //Entites own collision proxies, but are not entries themselves.
                if ((broadPhaseEntryOwner = (IBroadPhaseEntryOwner)spaceObject) != null)
                {
                    BroadPhase.Add(broadPhaseEntryOwner.Entry);
                    broadPhaseBoundingBoxUpdateable = broadPhaseEntryOwner.Entry as MobileCollidable;
                    if (broadPhaseBoundingBoxUpdateable != null)
                    {
                        BoundingBoxUpdater.Add(broadPhaseBoundingBoxUpdateable);
                    }
                }

                if ((solverUpdateable = (SolverUpdateable)spaceObject) != null)
                {
                    Solver.Add(solverUpdateable);
                }

                if ((integrable = (IPositionUpdateable)spaceObject) != null)
                {
                    PositionUpdater.Add(integrable);
                }

                if ((entity = (Entity)spaceObject) != null)
                {
                    BufferedStates.Add(entity);
                }

                if ((deferredEventCreator = (IDeferredEventCreator)spaceObject) != null)
                {
                    DeferredEventDispatcher.AddEventCreator(deferredEventCreator);
                }

                if ((deferredEventCreatorOwner = (IDeferredEventCreatorOwner)spaceObject) != null)
                {
                    DeferredEventDispatcher.AddEventCreator(deferredEventCreatorOwner.EventCreator);
                }

                //Updateable stages.
                if ((duringForcesUpdateable = (IDuringForcesUpdateable)spaceObject) != null)
                {
                    DuringForcesUpdateables.Add(duringForcesUpdateable);
                }

                if ((beforeNarrowPhaseUpdateable = (IBeforeNarrowPhaseUpdateable)spaceObject) != null)
                {
                    BeforeNarrowPhaseUpdateables.Add(beforeNarrowPhaseUpdateable);
                }

                if ((beforeSolverUpdateable = (IBeforeSolverUpdateable)spaceObject) != null)
                {
                    BeforeSolverUpdateables.Add(beforeSolverUpdateable);
                }

                if ((beforePositionUpdateUpdateable = (IBeforePositionUpdateUpdateable)spaceObject) != null)
                {
                    BeforePositionUpdateUpdateables.Add(beforePositionUpdateUpdateable);
                }

                if ((endOfStepUpdateable = (IEndOfTimeStepUpdateable)spaceObject) != null)
                {
                    EndOfTimeStepUpdateables.Add(endOfStepUpdateable);
                }

                if ((endOfFrameUpdateable = (IEndOfFrameUpdateable)spaceObject) != null)
                {
                    EndOfFrameUpdateables.Add(endOfFrameUpdateable);
                }

                spaceObject.OnAdditionToSpace(this);
            }
            catch
            {
                try
                {
                    if (simulationIslandMember != null)
                    {
                        DeactivationManager.Remove(simulationIslandMember);
                    }

                    if (simulationIslandMemberOwner != null)
                    {
                        DeactivationManager.Remove(simulationIslandMemberOwner.ActivityInformation);
                    }

                    //Go through each stage, adding the space object to it if necessary.
                    if (velocityUpdateable != null)
                    {
                        ForceUpdater.Remove(velocityUpdateable);
                    }

                    if (boundingBoxUpdateable != null)
                    {
                        BoundingBoxUpdater.Remove(boundingBoxUpdateable);
                    }

                    if (broadPhaseEntry != null)
                    {
                        BroadPhase.Remove(broadPhaseEntry);
                    }

                    //Entites own collision proxies, but are not entries themselves.
                    if (broadPhaseEntryOwner != null)
                    {
                        BroadPhase.Remove(broadPhaseEntryOwner.Entry);
                        if (broadPhaseBoundingBoxUpdateable != null)
                        {
                            BoundingBoxUpdater.Remove(broadPhaseBoundingBoxUpdateable);
                        }
                    }

                    if (solverUpdateable != null)
                    {
                        Solver.Remove(solverUpdateable);
                    }

                    if (integrable != null)
                    {
                        PositionUpdater.Remove(integrable);
                    }

                    if (entity != null)
                    {
                        BufferedStates.Remove(entity);
                    }

                    if (deferredEventCreator != null)
                    {
                        DeferredEventDispatcher.RemoveEventCreator(deferredEventCreator);
                    }

                    if (deferredEventCreatorOwner != null)
                    {
                        DeferredEventDispatcher.RemoveEventCreator(deferredEventCreatorOwner.EventCreator);
                    }

                    //Updateable stages.
                    if (duringForcesUpdateable != null)
                    {
                        DuringForcesUpdateables.Remove(duringForcesUpdateable);
                    }

                    if (beforeNarrowPhaseUpdateable != null)
                    {
                        BeforeNarrowPhaseUpdateables.Remove(beforeNarrowPhaseUpdateable);
                    }

                    if (beforeSolverUpdateable != null)
                    {
                        BeforeSolverUpdateables.Remove(beforeSolverUpdateable);
                    }

                    if (beforePositionUpdateUpdateable != null)
                    {
                        BeforePositionUpdateUpdateables.Remove(beforePositionUpdateUpdateable);
                    }

                    if (endOfStepUpdateable != null)
                    {
                        EndOfTimeStepUpdateables.Remove(endOfStepUpdateable);
                    }

                    if (endOfFrameUpdateable != null)
                    {
                        EndOfFrameUpdateables.Remove(endOfFrameUpdateable);
                    }
                }
                catch
                {
                    spaceObject.Space = null;
                    spaceObject.OnRemovalFromSpace(this);
                    return false;
                }

                spaceObject.Space = null;
                spaceObject.OnRemovalFromSpace(this);
                return false;
            }

            return true;
            */
        }

        ///<summary>
        /// Removes a space object from the simulation.
        ///</summary>
        ///<param name="spaceObject">Space object to remove.</param>
        public bool Remove(ISpaceObject spaceObject)
        {
            if (spaceObject.Space != this)
                return false; // throw new ArgumentException("The object does not belong to this space; cannot remove it.");

            if (spaceObject is SimulationIslandMember simulationIslandMember)
            {
                DeactivationManager.Remove(simulationIslandMember);
            }

            if (spaceObject is ISimulationIslandMemberOwner simulationIslandMemberOwner)
            {
                DeactivationManager.Remove(simulationIslandMemberOwner.ActivityInformation);
            }

            //Go through each stage, removing the space object from it if necessary.
            if (spaceObject is IForceUpdateable velocityUpdateable)
            {
                ForceUpdater.Remove(velocityUpdateable);
            }

            if (spaceObject is MobileCollidable boundingBoxUpdateable)
            {
                BoundingBoxUpdater.Remove(boundingBoxUpdateable);
            }

            if (spaceObject is BroadPhaseEntry broadPhaseEntry)
            {
                BroadPhase.Remove(broadPhaseEntry);
            }

            //Entites own collision proxies, but are not entries themselves.
            if (spaceObject is IBroadPhaseEntryOwner broadPhaseEntryOwner)
            {
                BroadPhase.Remove(broadPhaseEntryOwner.Entry);
                boundingBoxUpdateable = broadPhaseEntryOwner.Entry as MobileCollidable;
                if (boundingBoxUpdateable != null)
                {
                    BoundingBoxUpdater.Remove(boundingBoxUpdateable);
                }
            }

            if (spaceObject is SolverUpdateable solverUpdateable)
            {
                Solver.Remove(solverUpdateable);
            }

            if (spaceObject is IPositionUpdateable integrable)
            {
                PositionUpdater.Remove(integrable);
            }

            if (spaceObject is Entity entity)
            {
                BufferedStates.Remove(entity);
            }

            if (spaceObject is IDeferredEventCreator deferredEventCreator)
            {
                DeferredEventDispatcher.RemoveEventCreator(deferredEventCreator);
            }

            if (spaceObject is IDeferredEventCreatorOwner deferredEventCreatorOwner)
            {
                DeferredEventDispatcher.RemoveEventCreator(deferredEventCreatorOwner.EventCreator);
            }

            //Updateable stages.
            if (spaceObject is IDuringForcesUpdateable duringForcesUpdateable)
            {
                DuringForcesUpdateables.Remove(duringForcesUpdateable);
            }

            if (spaceObject is IBeforeNarrowPhaseUpdateable beforeNarrowPhaseUpdateable)
            {
                BeforeNarrowPhaseUpdateables.Remove(beforeNarrowPhaseUpdateable);
            }

            if (spaceObject is IBeforeSolverUpdateable beforeSolverUpdateable)
            {
                BeforeSolverUpdateables.Remove(beforeSolverUpdateable);
            }


            if (spaceObject is IBeforePositionUpdateUpdateable beforePositionUpdateUpdateable)
            {
                BeforePositionUpdateUpdateables.Remove(beforePositionUpdateUpdateable);
            }

            if (spaceObject is IEndOfTimeStepUpdateable endOfStepUpdateable)
            {
                EndOfTimeStepUpdateables.Remove(endOfStepUpdateable);
            }

            if (spaceObject is IEndOfFrameUpdateable endOfFrameUpdateable)
            {
                EndOfFrameUpdateables.Remove(endOfFrameUpdateable);
            }

            spaceObject.Space = null;
            spaceObject.OnRemovalFromSpace(this);
            return true;
        }

#if PROFILE
        /// <summary>
        /// Gets the time it took to perform the previous time step.
        /// </summary>
        public double Time
            => (end - start) / (double)Stopwatch.Frequency;

        private long start, end;
#endif

        void DoTimeStep()
        {
#if PROFILE
            start = Stopwatch.GetTimestamp();
#endif
            SpaceObjectBuffer.Update();
            EntityStateWriteBuffer.Update();
            DeactivationManager.Update();
            ForceUpdater.Update();
            DuringForcesUpdateables.Update();
            BoundingBoxUpdater.Update();
            BroadPhase.Update();
            BeforeNarrowPhaseUpdateables.Update();
            NarrowPhase.Update();
            BeforeSolverUpdateables.Update();
            Solver.Update();
            BeforePositionUpdateUpdateables.Update();
            PositionUpdater.Update();
            BufferedStates.ReadBuffers.Update();
            DeferredEventDispatcher.Update();
            EndOfTimeStepUpdateables.Update();
#if PROFILE
            end = Stopwatch.GetTimestamp();
#endif
        }

        ///<summary>
        /// Performs a single timestep.
        ///</summary>
        public void Update()
        {
            DoTimeStep();
            EndOfFrameUpdateables.Update();
        }

        /// <summary>
        /// Performs as many timesteps as necessary to get as close to the elapsed time as possible.
        /// </summary>
        /// <param name="dt">Elapsed time from the previous frame.</param>
        public void Update(float dt)
        {
            TimeStepSettings.AccumulatedTime += dt;
            for (int i = 0; i < TimeStepSettings.MaximumTimeStepsPerFrame; i++)
            {
                if (TimeStepSettings.AccumulatedTime >= TimeStepSettings.TimeStepDuration)
                {
                    TimeStepSettings.AccumulatedTime -= TimeStepSettings.TimeStepDuration;
                    DoTimeStep();
                }
                else
                {
                    break;
                }
            }

            BufferedStates.InterpolatedStates.BlendAmount = TimeStepSettings.AccumulatedTime / TimeStepSettings.TimeStepDuration;
            BufferedStates.InterpolatedStates.Update();
            EndOfFrameUpdateables.Update();
        }

        /// <summary>
        /// Tests a ray against the space.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="result">Hit data of the ray, if any.</param>
        /// <returns>Whether or not the ray hit anything.</returns>
        public bool RayCast(Ray ray, out RayCastResult result)
            => RayCast(ray, float.MaxValue, out result);

        /// <summary>
        /// Tests a ray against the space.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="filter">Delegate to prune out hit candidates before performing a ray cast against them. Return true from the filter to process an entry or false to ignore the entry.</param>
        /// <param name="result">Hit data of the ray, if any.</param>
        /// <returns>Whether or not the ray hit anything.</returns>
        public bool RayCast(Ray ray, Func<BroadPhaseEntry, bool> filter, out RayCastResult result)
            => RayCast(ray, float.MaxValue, filter, out result);

        /// <summary>
        /// Tests a ray against the space.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        /// <param name="result">Hit data of the ray, if any.</param>
        /// <returns>Whether or not the ray hit anything.</returns>
        public bool RayCast(Ray ray, float maximumLength, out RayCastResult result)
        {
            var resultsList = PhysicsResources.GetRayCastResultList();
            bool didHit = RayCast(ray, maximumLength, resultsList);
            result = resultsList.Elements[0];
            for (int i = 1; i < resultsList.Count; i++)
            {
                RayCastResult candidate = resultsList.Elements[i];
                if (candidate.HitData.T < result.HitData.T)
                    result = candidate;
            }
            PhysicsResources.GiveBack(resultsList);

            return didHit;
        }

        /// <summary>
        /// Tests a ray against the space.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        /// <param name="filter">Delegate to prune out hit candidates before performing a ray cast against them. Return true from the filter to process an entry or false to ignore the entry.</param>
        /// <param name="result">Hit data of the ray, if any.</param>
        /// <returns>Whether or not the ray hit anything.</returns>
        public bool RayCast(Ray ray, float maximumLength, Func<BroadPhaseEntry, bool> filter, out RayCastResult result)
        {
            var resultsList = PhysicsResources.GetRayCastResultList();
            bool didHit = RayCast(ray, maximumLength, filter, resultsList);
            result = resultsList.Elements[0];
            for (int i = 1; i < resultsList.Count; i++)
            {
                RayCastResult candidate = resultsList.Elements[i];
                if (candidate.HitData.T < result.HitData.T)
                    result = candidate;
            }
            PhysicsResources.GiveBack(resultsList);

            return didHit;
        }

        /// <summary>
        /// Tests a ray against the space, possibly returning multiple hits.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        /// <param name="outputRayCastResults">Hit data of the ray, if any.</param>
        /// <returns>Whether or not the ray hit anything.</returns>
        public bool RayCast(Ray ray, float maximumLength, IList<RayCastResult> outputRayCastResults)
        {
            var outputIntersections = PhysicsResources.GetBroadPhaseEntryList();
            if (BroadPhase.QueryAccelerator.RayCast(ray, maximumLength, outputIntersections))
            {

                for (int i = 0; i < outputIntersections.Count; i++)
                {
                    BroadPhaseEntry candidate = outputIntersections.Elements[i];
                    if (candidate.RayCast(ray, maximumLength, out RayHit rayHit))
                    {
                        outputRayCastResults.Add(new RayCastResult(rayHit, candidate));
                    }
                }
            }
            PhysicsResources.GiveBack(outputIntersections);
            return outputRayCastResults.Count > 0;
        }

        /// <summary>
        /// Tests a ray against the space, possibly returning multiple hits.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        /// <param name="filter">Delegate to prune out hit candidates before performing a cast against them. Return true from the filter to process an entry or false to ignore the entry.</param>
        /// <param name="outputRayCastResults">Hit data of the ray, if any.</param>
        /// <returns>Whether or not the ray hit anything.</returns>
        public bool RayCast(Ray ray, float maximumLength, Func<BroadPhaseEntry, bool> filter, IList<RayCastResult> outputRayCastResults)
        {
            var outputIntersections = PhysicsResources.GetBroadPhaseEntryList();
            if (BroadPhase.QueryAccelerator.RayCast(ray, maximumLength, outputIntersections))
            {

                for (int i = 0; i < outputIntersections.Count; i++)
                {
                    BroadPhaseEntry candidate = outputIntersections.Elements[i];
                    if (candidate.RayCast(ray, maximumLength, filter, out RayHit rayHit))
                    {
                        outputRayCastResults.Add(new RayCastResult(rayHit, candidate));
                    }
                }
            }
            PhysicsResources.GiveBack(outputIntersections);
            return outputRayCastResults.Count > 0;
        }

        /// <summary>
        /// <para>Casts a convex shape against the space.</para>
        /// <para>Convex casts are sensitive to length; avoid extremely long convex casts for better stability and performance.</para>
        /// </summary>
        /// <param name="castShape">Shape to cast.</param>
        /// <param name="startingTransform">Initial transform of the shape.</param>
        /// <param name="sweep">Sweep to apply to the shape. Avoid extremely long convex casts for better stability and performance.</param>
        /// <param name="castResult">Hit data, if any.</param>
        /// <returns>Whether or not the cast hit anything.</returns>
        public bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, out RayCastResult castResult)
        {
            var castResults = PhysicsResources.GetRayCastResultList();
            bool didHit = ConvexCast(castShape, ref startingTransform, ref sweep, castResults);
            castResult = castResults.Elements[0];
            for (int i = 1; i < castResults.Count; i++)
            {
                RayCastResult candidate = castResults.Elements[i];
                if (candidate.HitData.T < castResult.HitData.T)
                    castResult = candidate;
            }
            PhysicsResources.GiveBack(castResults);
            return didHit;
        }

        /// <summary>
        /// <para>Casts a convex shape against the space.</para>
        /// <para>Convex casts are sensitive to length; avoid extremely long convex casts for better stability and performance.</para>
        /// </summary>
        /// <param name="castShape">Shape to cast.</param>
        /// <param name="startingTransform">Initial transform of the shape.</param>
        /// <param name="sweep">Sweep to apply to the shape. Avoid extremely long convex casts for better stability and performance.</param>
        /// <param name="filter">Delegate to prune out hit candidates before performing a cast against them. Return true from the filter to process an entry or false to ignore the entry.</param>
        /// <param name="castResult">Hit data, if any.</param>
        /// <returns>Whether or not the cast hit anything.</returns>
        public bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, Func<BroadPhaseEntry, bool> filter, out RayCastResult castResult)
        {
            var castResults = PhysicsResources.GetRayCastResultList();
            bool didHit = ConvexCast(castShape, ref startingTransform, ref sweep, filter, castResults);
            castResult = castResults.Elements[0];
            for (int i = 1; i < castResults.Count; i++)
            {
                RayCastResult candidate = castResults.Elements[i];
                if (candidate.HitData.T < castResult.HitData.T)
                    castResult = candidate;
            }
            PhysicsResources.GiveBack(castResults);
            return didHit;
        }

        /// <summary>
        /// <para>Casts a convex shape against the space.</para>
        /// <para>Convex casts are sensitive to length; avoid extremely long convex casts for better stability and performance.</para>
        /// </summary>
        /// <param name="castShape">Shape to cast.</param>
        /// <param name="startingTransform">Initial transform of the shape.</param>
        /// <param name="sweep">Sweep to apply to the shape. Avoid extremely long convex casts for better stability and performance.</param>
        /// <param name="outputCastResults">Hit data, if any.</param>
        /// <returns>Whether or not the cast hit anything.</returns>
        public bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, IList<RayCastResult> outputCastResults)
        {
            var overlappedElements = PhysicsResources.GetBroadPhaseEntryList();
            castShape.GetSweptBoundingBox(ref startingTransform, ref sweep, out BoundingBox boundingBox);

            BroadPhase.QueryAccelerator.GetEntries(boundingBox, overlappedElements);
            for (int i = 0; i < overlappedElements.Count; ++i)
            {
                if (overlappedElements.Elements[i].ConvexCast(castShape, ref startingTransform, ref sweep, out RayHit hit))
                {
                    outputCastResults.Add(new RayCastResult { HitData = hit, HitObject = overlappedElements.Elements[i] });
                }
            }
            PhysicsResources.GiveBack(overlappedElements);
            return outputCastResults.Count > 0;
        }

        /// <summary>
        /// <para>Casts a convex shape against the space.</para>
        /// <para>Convex casts are sensitive to length; avoid extremely long convex casts for better stability and performance.</para>
        /// </summary>
        /// <param name="castShape">Shape to cast.</param>
        /// <param name="startingTransform">Initial transform of the shape.</param>
        /// <param name="sweep">Sweep to apply to the shape. Avoid extremely long convex casts for better stability and performance.</param>
        /// <param name="filter">Delegate to prune out hit candidates before performing a cast against them. Return true from the filter to process an entry or false to ignore the entry.</param>
        /// <param name="outputCastResults">Hit data, if any.</param>
        /// <returns>Whether or not the cast hit anything.</returns>
        public bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, Func<BroadPhaseEntry, bool> filter, IList<RayCastResult> outputCastResults)
        {
            var overlappedElements = PhysicsResources.GetBroadPhaseEntryList();
            castShape.GetSweptBoundingBox(ref startingTransform, ref sweep, out BoundingBox boundingBox);

            BroadPhase.QueryAccelerator.GetEntries(boundingBox, overlappedElements);
            for (int i = 0; i < overlappedElements.Count; ++i)
            {
                if (overlappedElements.Elements[i].ConvexCast(castShape, ref startingTransform, ref sweep, filter, out RayHit hit))
                {
                    outputCastResults.Add(new RayCastResult { HitData = hit, HitObject = overlappedElements.Elements[i] });
                }
            }
            PhysicsResources.GiveBack(overlappedElements);
            return outputCastResults.Count > 0;
        }
    }
}
