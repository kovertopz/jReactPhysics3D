package net.smert.jreactphysics3d.engine;

import java.util.List;
import java.util.Map;
import java.util.Set;
import net.smert.jreactphysics3d.body.BodyIndexPair;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.collision.BroadPhasePair;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.configuration.ContactsPositionCorrectionTechnique;
import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.configuration.JointsPositionCorrectionTechnique;
import net.smert.jreactphysics3d.constraint.BallAndSocketJoint;
import net.smert.jreactphysics3d.constraint.BallAndSocketJointInfo;
import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
import net.smert.jreactphysics3d.constraint.FixedJoint;
import net.smert.jreactphysics3d.constraint.FixedJointInfo;
import net.smert.jreactphysics3d.constraint.HingeJoint;
import net.smert.jreactphysics3d.constraint.HingeJointInfo;
import net.smert.jreactphysics3d.constraint.Joint;
import net.smert.jreactphysics3d.constraint.JointInfo;
import net.smert.jreactphysics3d.constraint.JointListElement;
import net.smert.jreactphysics3d.constraint.SliderJoint;
import net.smert.jreactphysics3d.constraint.SliderJointInfo;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a dynamics world. This class inherits from the CollisionWorld class. In a dynamics world,
 * bodies can collide and their movements are simulated using the laws of physics.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class DynamicsWorld extends CollisionWorld {

    /// Timer of the physics engine
    protected Timer mTimer;

    /// Contact solver
    protected ContactSolver mContactSolver;

    /// Constraint solver
    protected ConstraintSolver mConstraintSolver;

    /// Number of iterations for the velocity solver of the Sequential Impulses technique
    protected int mNbVelocitySolverIterations;

    /// Number of iterations for the position solver of the Sequential Impulses technique
    protected int mNbPositionSolverIterations;

    /// True if the spleeping technique for inactive bodies is enabled
    protected boolean mIsSleepingEnabled;

    /// All the rigid bodies of the physics world
    protected Set<RigidBody> mRigidBodies;

    /// All the contact constraints
    // TODO : Remove this variable (we will use the ones in the island now)
    protected List<ContactManifold> mContactManifolds;

    /// All the joints of the world
    protected Set<Joint> mJoints;

    /// Gravity vector of the world
    protected Vector3 mGravity;

    /// True if the gravity force is on
    protected boolean mIsGravityEnabled;

    /// Array of constrained linear velocities (state of the linear velocities
    /// after solving the constraints)
    protected Vector3 mConstrainedLinearVelocities;

    /// Array of constrained angular velocities (state of the angular velocities
    /// after solving the constraints)
    protected Vector3 mConstrainedAngularVelocities;

    /// Split linear velocities for the position contact solver (split impulse)
    protected Vector3 mSplitLinearVelocities;

    /// Split angular velocities for the position contact solver (split impulse)
    protected Vector3 mSplitAngularVelocities;

    /// Array of constrained rigid bodies position (for position error correction)
    protected List<Vector3> mConstrainedPositions;

    /// Array of constrained rigid bodies orientation (for position error correction)
    protected List<Quaternion> mConstrainedOrientations;

    /// Map body to their index in the constrained velocities array
    protected Map<RigidBody, Integer> mMapBodyToConstrainedVelocityIndex;

    /// Number of islands in the world
    protected int mNbIslands;

    /// Current allocated capacity for the islands
    protected int mNbIslandsCapacity;

    /// Array with all the islands of awaken bodies
    protected Island mIslands;

    /// Current allocated capacity for the bodies
    protected int mNbBodiesCapacity;

    /// Sleep linear velocity threshold
    protected float mSleepLinearVelocity;

    /// Sleep angular velocity threshold
    protected float mSleepAngularVelocity;

    /// Time (in seconds) before a body is put to sleep if its velocity
    /// becomes smaller than the sleep velocity.
    protected float mTimeBeforeSleep;

    /// Pointer to an event listener object
    protected EventListener mEventListener;

    /// Private copy-constructor
    protected DynamicsWorld(DynamicsWorld world) {
    }

    /// Private assignment operator
    protected DynamicsWorld operatorEqual(DynamicsWorld world) {
        return this;
    }

    // Integrate position and orientation of the rigid bodies.
    /// The positions and orientations of the bodies are integrated using
    /// the sympletic Euler time stepping scheme.
    protected void integrateRigidBodiesPositions() {

        PROFILE("DynamicsWorld::integrateRigidBodiesPositions()");

        float dt = (float) mTimer.getTimeStep();

        // For each island of the world
        for (int i = 0; i < mNbIslands; i++) {

            RigidBody bodies = mIslands[i].getBodies();

            // For each body of the island
            for (int b = 0; b < mIslands[i].getNbBodies(); b++) {

                // If the body is allowed to move
                if (bodies[b].isMotionEnabled()) {

                    // Get the constrained velocity
                    int indexArray = mMapBodyToConstrainedVelocityIndex.find(bodies[b]).second;
                    Vector3 newLinVelocity = mConstrainedLinearVelocities[indexArray];
                    Vector3 newAngVelocity = mConstrainedAngularVelocities[indexArray];

                    // Update the linear and angular velocity of the body
                    bodies[b].setLinearVelocity(newLinVelocity);
                    bodies[b].setAngularVelocity(newAngVelocity);

                    // Add the split impulse velocity from Contact Solver (only used
                    // to update the position)
                    if (mContactSolver.isSplitImpulseActive()) {

                        newLinVelocity += mSplitLinearVelocities[indexArray];
                        newAngVelocity += mSplitAngularVelocities[indexArray];
                    }

                    // Get current position and orientation of the body
                    Vector3 currentPosition = bodies[b].getTransform().getPosition();
                    Quaternion currentOrientation = bodies[b].getTransform().getOrientation();

                    // Compute the new position of the body
                    Vector3 newPosition = currentPosition + newLinVelocity * dt;
                    Quaternion newOrientation = currentOrientation + Quaternion(0, newAngVelocity) * currentOrientation * 0.5f * dt;

                    // Update the Transform of the body
                    Transform newTransform(newPosition
                        , newOrientation.getUnit()
                        );
                        bodies[b].setTransform(newTransform);
                }
            }
        }
    }

    // Update the AABBs of the bodies
    protected void updateRigidBodiesAABB() {

        PROFILE("DynamicsWorld::updateRigidBodiesAABB()");

        // For each rigid body of the world
        Set<RigidBody> it;
        for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

            // If the body has moved
            if (it.mHasMoved) {

                // Update the AABB of the rigid body
                it.updateAABB();
            }
        }
    }

    // Reset the external force and torque applied to the bodies
    protected void resetBodiesForceAndTorque() {

        // For each body of the world
        Set<RigidBody> it;
        for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {
            it.mExternalForce.setToZero();
            it.mExternalTorque.setToZero();
        }
    }

    /// Update the position and orientation of a body
    protected void updatePositionAndOrientationOfBody(RigidBody body, Vector3 newLinVelocity,
            Vector3 newAngVelocity) {
    }

    // Compute and set the interpolation factor to all bodies
    protected void setInterpolationFactorToAllBodies() {

        PROFILE("DynamicsWorld::setInterpolationFactorToAllBodies()");

        // Compute the interpolation factor
        float factor = mTimer.computeInterpolationFactor();
        assert (factor >= 0.0f && factor <= 1.0f);

        // Set the factor to all bodies
        Set<RigidBody> it;
        for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

            it.setInterpolationFactor(factor);
        }
    }

    // Initialize the bodies velocities arrays for the next simulation step.
    protected void initVelocityArrays() {

        // Allocate memory for the bodies velocity arrays
        int nbBodies = mRigidBodies.size();
        if (mNbBodiesCapacity != nbBodies && nbBodies > 0) {
            if (mNbBodiesCapacity > 0) {
                //delete[] mSplitLinearVelocities;
                //delete[] mSplitAngularVelocities;
            }
            mNbBodiesCapacity = nbBodies;
            mSplitLinearVelocities = new Vector3[mNbBodiesCapacity];
            mSplitAngularVelocities = new Vector3[mNbBodiesCapacity];
            mConstrainedLinearVelocities = new Vector3[mNbBodiesCapacity];
            mConstrainedAngularVelocities = new Vector3[mNbBodiesCapacity];
            assert (mSplitLinearVelocities != null);
            assert (mSplitAngularVelocities != null);
            assert (mConstrainedLinearVelocities != null);
            assert (mConstrainedAngularVelocities != null);
        }

        // Reset the velocities arrays
        for (int i = 0; i < mNbBodiesCapacity; i++) {
            mSplitLinearVelocities[i].setToZero();
            mSplitAngularVelocities[i].setToZero();
        }

        // Initialize the map of body indexes in the velocity arrays
        mMapBodyToConstrainedVelocityIndex.clear();
        Set<RigidBody> it;
        int indexBody = 0;
        for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

            // Add the body into the map
            //mMapBodyToConstrainedVelocityIndex.insert(std::make_pair<RigidBody,
            //                                          int>(it, indexBody));
            indexBody++;
        }
    }

    // Integrate the velocities of rigid bodies.
    /// This method only set the temporary velocities but does not update
    /// the actual velocitiy of the bodies. The velocities updated in this method
    /// might violate the constraints and will be corrected in the constraint and
    /// contact solver.
    protected void integrateRigidBodiesVelocities() {

        PROFILE("DynamicsWorld::integrateRigidBodiesVelocities()");

        // Initialize the bodies velocity arrays
        initVelocityArrays();

        float dt = (float) mTimer.getTimeStep();

        // For each island of the world
        for (int i = 0; i < mNbIslands; i++) {

            RigidBody bodies = mIslands[i].getBodies();

            // For each body of the island
            for (int b = 0; b < mIslands[i].getNbBodies(); b++) {

                // Insert the body into the map of constrained velocities
                int indexBody = mMapBodyToConstrainedVelocityIndex.find(bodies[b]).second;

                assert (mSplitLinearVelocities[indexBody] == new Vector3(0.0f, 0.0f, 0.0f));
                assert (mSplitAngularVelocities[indexBody] == new Vector3(0.0f, 0.0f, 0.0f));

                // If the body is allowed to move
                if (bodies[b].isMotionEnabled()) {

                    // Integrate the external force to get the new velocity of the body
                    mConstrainedLinearVelocities[indexBody] = bodies[b].getLinearVelocity()
                            + dt * bodies[b].getMassInverse() * bodies[b].mExternalForce;
                    mConstrainedAngularVelocities[indexBody] = bodies[b].getAngularVelocity()
                            + dt * bodies[b].getInertiaTensorInverseWorld() * bodies[b].mExternalTorque;

                    // If the gravity has to be applied to this rigid body
                    if (bodies[b].isGravityEnabled() && mIsGravityEnabled) {

                        // Integrate the gravity force
                        mConstrainedLinearVelocities[indexBody] += dt * bodies[b].getMassInverse() * bodies[b].getMass() * mGravity;
                    }

                    // Apply the velocity damping
                    // Damping force : F_c = -c' * v (c=damping factor)
                    // Equation      : m * dv/dt = -c' * v
                    //                 => dv/dt = -c * v (with c=c'/m)
                    //                 => dv/dt + c * v = 0
                    // Solution      : v(t) = v0 * e^(-c * t)
                    //                 => v(t + dt) = v0 * e^(-c(t + dt))
                    //                              = v0 * e^(-ct) * e^(-c * dt)
                    //                              = v(t) * e^(-c * dt)
                    //                 => v2 = v1 * e^(-c * dt)
                    // Using Taylor Serie for e^(-x) : e^x ~ 1 + x + x^2/2! + ...
                    //                              => e^(-x) ~ 1 - x
                    //                 => v2 = v1 * (1 - c * dt)
                    float linDampingFactor = bodies[b].getLinearDamping();
                    float angDampingFactor = bodies[b].getAngularDamping();
                    float linearDamping = clamp(1.0f - dt * linDampingFactor, 0.0f, 1.0f);
                    float angularDamping = clamp(1.0f - dt * angDampingFactor, 0.0f, 1.0f);
                    mConstrainedLinearVelocities[indexBody] *= clamp(linearDamping, 0.0f, 1.0f);
                    mConstrainedAngularVelocities[indexBody] *= clamp(angularDamping, 0.0f, 1.0f);

                    // Update the old Transform of the body
                    bodies[b].updateOldTransform();
                }

                indexBody++;
            }
        }
    }

    // Solve the contacts and constraints
    protected void solveContactsAndConstraints() {

        PROFILE("DynamicsWorld::solveContactsAndConstraints()");

        // Get the current time step
        float dt = (float) mTimer.getTimeStep();

        // Set the velocities arrays
        mContactSolver.setSplitVelocitiesArrays(mSplitLinearVelocities, mSplitAngularVelocities);
        mContactSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities,
                mConstrainedAngularVelocities);
        mConstraintSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities,
                mConstrainedAngularVelocities);

        // ---------- Solve velocity constraints for joints and contacts ---------- //
        // For each island of the world
        for (int islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

            // Check if there are contacts and constraints to solve
            boolean isConstraintsToSolve = mIslands[islandIndex].getNbJoints() > 0;
            boolean isContactsToSolve = mIslands[islandIndex].getNbContactManifolds() > 0;
            if (!isConstraintsToSolve && !isContactsToSolve) {
                continue;
            }

            // If there are contacts in the current island
            if (isContactsToSolve) {

                // Initialize the solver
                mContactSolver.initializeForIsland(dt, mIslands[islandIndex]);

                // Warm start the contact solver
                mContactSolver.warmStart();
            }

            // If there are constraints
            if (isConstraintsToSolve) {

                // Initialize the constraint solver
                mConstraintSolver.initializeForIsland(dt, mIslands[islandIndex]);
            }

            // For each iteration of the velocity solver
            for (int i = 0; i < mNbVelocitySolverIterations; i++) {

                // Solve the constraints
                if (isConstraintsToSolve) {
                    mConstraintSolver.solveVelocityConstraints(mIslands[islandIndex]);
                }

                // Solve the contacts
                if (isContactsToSolve) {
                    mContactSolver.solve();
                }
            }

            // Cache the lambda values in order to use them in the next
            // step and cleanup the contact solver
            if (isContactsToSolve) {
                mContactSolver.storeImpulses();
                mContactSolver.cleanup();
            }
        }
    }

    // Solve the position error correction of the constraints
    protected void solvePositionCorrection() {

        PROFILE("DynamicsWorld::solvePositionCorrection()");

        // Do not continue if there is no constraints
        if (mJoints.empty()) {
            return;
        }

        // ---------- Get the position/orientation of the rigid bodies ---------- //
        // TODO : Use better memory allocation here
        mConstrainedPositions = List < Vector3 > (mRigidBodies.size());
        mConstrainedOrientations = List < Quaternion > (mRigidBodies.size());

        // For each island of the world
        for (int islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

            // For each body of the island
            RigidBody bodies = mIslands[islandIndex].getBodies();
            for (int b = 0; b < mIslands[islandIndex].getNbBodies(); b++) {

                int index = mMapBodyToConstrainedVelocityIndex.find(bodies[b]).second;

                // Get the position/orientation of the rigid body
                Transform transform = bodies[b].getTransform();
                mConstrainedPositions[index] = transform.getPosition();
                mConstrainedOrientations[index] = transform.getOrientation();
            }

            // ---------- Solve the position error correction for the constraints ---------- //
            // For each iteration of the position (error correction) solver
            for (int i = 0; i < mNbPositionSolverIterations; i++) {

                // Solve the position constraints
                mConstraintSolver.solvePositionConstraints(mIslands[islandIndex]);
            }

            // ---------- Update the position/orientation of the rigid bodies ---------- //
            for (int b = 0; b < mIslands[islandIndex].getNbBodies(); b++) {

                int index = mMapBodyToConstrainedVelocityIndex.find(bodies[b]).second;

                // Get the new position/orientation of the body
                Vector3 newPosition = mConstrainedPositions[index];
                Quaternion newOrientation = mConstrainedOrientations[index];

                // Update the Transform of the body
                Transform newTransform(newPosition
                    , newOrientation.getUnit()
                    );
                    bodies[b].setTransform(newTransform);
            }
        }
    }

    /// Cleanup the constrained velocities array at each step
    protected void cleanupConstrainedVelocitiesArray() {
    }

    // Reset the boolean movement variable of each body
    protected void resetBodiesMovementVariable() {

        // For each rigid body
        for (Set<RigidBody> it = getRigidBodiesBeginIterator(); it != getRigidBodiesEndIterator(); it++) {

            // Set the hasMoved variable to false
            it.mHasMoved = false;
        }
    }

    // Compute the islands of awake bodies.
    /// An island is an isolated group of rigid bodies that have constraints (joints or contacts)
    /// between each other. This method computes the islands at each time step as follows: For each
    /// awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
    /// (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
    /// find all the bodies that are connected with it (the bodies that share joints or contacts with
    /// it). Then, we create an island with this group of connected bodies.
    protected void computeIslands() {

        PROFILE("DynamicsWorld::computeIslands()");

        int nbBodies = mRigidBodies.size();

        // Clear all the islands
        for (int i = 0; i < mNbIslands; i++) {
            // Call the island destructor
            //mIslands[i].Island::~Island();

            // Release the allocated memory for the island
            mMemoryAllocator.release(mIslands[i], sizeof(Island));
        }

        // Allocate and create the array of islands
        if (mNbIslandsCapacity != nbBodies && nbBodies > 0) {
            if (mNbIslandsCapacity > 0) {
                //mMemoryAllocator.release(mIslands, sizeof(Island*) * mNbIslandsCapacity);
            }
            mNbIslandsCapacity = nbBodies;
            //mIslands = (Island**)mMemoryAllocator.allocate(sizeof(Island*) * mNbIslandsCapacity);
        }
        mNbIslands = 0;

        // Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
        for (Set<RigidBody> it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {
            it.mIsAlreadyInIsland = false;
        }
        for (List<ContactManifold> it = mContactManifolds.begin();
                it != mContactManifolds.end(); ++it) {
            it.mIsAlreadyInIsland = false;
        }
        for (Set<Joint> it = mJoints.begin(); it != mJoints.end(); ++it) {
            it.mIsAlreadyInIsland = false;
        }

        // Create a stack (using an array) for the rigid bodies to visit during the Depth First Search
        int nbBytesStack = sizeof(RigidBody) * nbBodies;
        RigidBody stackBodiesToVisit = (RigidBody) mMemoryAllocator.allocate(nbBytesStack);

        // For each rigid body of the world
        for (Set<RigidBody> it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

            RigidBody body = it;

            // If the body has already been added to an island, we go to the next body
            if (body.mIsAlreadyInIsland) {
                continue;
            }

            // If the body is not moving, we go to the next body
            // TODO : When we will use STATIC bodies, we will need to take care of this case here
            if (!body.isMotionEnabled()) {
                continue;
            }

            // If the body is sleeping or inactive, we go to the next body
            if (body.isSleeping() || !body.isActive()) {
                continue;
            }

            // Reset the stack of bodies to visit
            int stackIndex = 0;
            stackBodiesToVisit[stackIndex] = body;
            stackIndex++;
            body.mIsAlreadyInIsland = true;

            // Create the new island
            //void* allocatedMemoryIsland = mMemoryAllocator.allocate(sizeof(Island));
            mIslands[mNbIslands] = new Island(nbBodies, mContactManifolds.size(), mJoints.size(), mMemoryAllocator);

            // While there are still some bodies to visit in the stack
            while (stackIndex > 0) {

                // Get the next body to visit from the stack
                stackIndex--;
                RigidBody bodyToVisit = stackBodiesToVisit[stackIndex];
                assert (bodyToVisit.isActive());

                // Awake the body if it is slepping
                bodyToVisit.setIsSleeping(false);

                // Add the body into the island
                mIslands[mNbIslands].addBody(bodyToVisit);

                // If the current body is not moving, we do not want to perform the DFS
                // search across that body
                if (!bodyToVisit.isMotionEnabled()) {
                    continue;
                }

                // For each contact manifold in which the current body is involded
                ContactManifoldListElement contactElement;
                for (contactElement = bodyToVisit.mContactManifoldsList; contactElement != null;
                        contactElement = contactElement.next) {

                    ContactManifold contactManifold = contactElement.contactManifold;

                    // Check if the current contact manifold has already been added into an island
                    if (contactManifold.isAlreadyInIsland()) {
                        continue;
                    }

                    // Add the contact manifold into the island
                    mIslands[mNbIslands].addContactManifold(contactManifold);
                    contactManifold.mIsAlreadyInIsland = true;

                    // Get the other body of the contact manifold
                    RigidBody body1 = dynamic_cast < RigidBody > (contactManifold.getBody1());
                    RigidBody body2 = dynamic_cast < RigidBody > (contactManifold.getBody2());
                    RigidBody otherBody = (body1.getID() == bodyToVisit.getID()) ? body2 : body1;

                    // Check if the other body has already been added to the island
                    if (otherBody.mIsAlreadyInIsland) {
                        continue;
                    }

                    // Insert the other body into the stack of bodies to visit
                    stackBodiesToVisit[stackIndex] = otherBody;
                    stackIndex++;
                    otherBody.mIsAlreadyInIsland = true;
                }

                // For each joint in which the current body is involved
                JointListElement jointElement;
                for (jointElement = bodyToVisit.mJointsList; jointElement != null;
                        jointElement = jointElement.next) {

                    Joint joint = jointElement.joint;

                    // Check if the current joint has already been added into an island
                    if (joint.isAlreadyInIsland()) {
                        continue;
                    }

                    // Add the joint into the island
                    mIslands[mNbIslands].addJoint(joint);
                    joint.mIsAlreadyInIsland = true;

                    // Get the other body of the contact manifold
                    RigidBody body1 = dynamic_cast < RigidBody > (joint.getBody1());
                    RigidBody body2 = dynamic_cast < RigidBody > (joint.getBody2());
                    RigidBody otherBody = (body1.getID() == bodyToVisit.getID()) ? body2 : body1;

                    // Check if the other body has already been added to the island
                    if (otherBody.mIsAlreadyInIsland) {
                        continue;
                    }

                    // Insert the other body into the stack of bodies to visit
                    stackBodiesToVisit[stackIndex] = otherBody;
                    stackIndex++;
                    otherBody.mIsAlreadyInIsland = true;
                }
            }

            // Reset the isAlreadyIsland variable of the static bodies so that they
            // can also be included in the other islands
            for (int i = 0; i < mIslands[mNbIslands].mNbBodies; i++) {

                if (!mIslands[mNbIslands].mBodies[i].isMotionEnabled()) {
                    mIslands[mNbIslands].mBodies[i].mIsAlreadyInIsland = false;
                }
            }

            mNbIslands++;
        }

        // Release the allocated memory for the stack of bodies to visit
        mMemoryAllocator.release(stackBodiesToVisit, nbBytesStack);
    }

    // Put bodies to sleep if needed.
    /// For each island, if all the bodies have been almost still for a long enough period of
    /// time, we put all the bodies of the island to sleep.
    protected void updateSleepingBodies() {

        PROFILE("DynamicsWorld::updateSleepingBodies()");

        float dt = (float) mTimer.getTimeStep();
        float sleepLinearVelocitySquare = mSleepLinearVelocity * mSleepLinearVelocity;
        float sleepAngularVelocitySquare = mSleepAngularVelocity * mSleepAngularVelocity;

        // For each island of the world
        for (int i = 0; i < mNbIslands; i++) {

            float minSleepTime = Defaults.DECIMAL_LARGEST;

            // For each body of the island
            RigidBody bodies = mIslands[i].getBodies();
            for (int b = 0; b < mIslands[i].getNbBodies(); b++) {

                // Skip static bodies
                if (!bodies[b].isMotionEnabled()) {
                    continue;
                }

                // If the body is velocity is large enough to stay awake
                if (bodies[b].getLinearVelocity().lengthSquare() > sleepLinearVelocitySquare
                        || bodies[b].getAngularVelocity().lengthSquare() > sleepAngularVelocitySquare
                        || !bodies[b].isAllowedToSleep()) {

                    // Reset the sleep time of the body
                    bodies[b].mSleepTime = 0.0f;
                    minSleepTime = 0.0f;
                } else {  // If the body velocity is bellow the sleeping velocity threshold

                    // Increase the sleep time
                    bodies[b].mSleepTime += dt;
                    if (bodies[b].mSleepTime < minSleepTime) {
                        minSleepTime = bodies[b].mSleepTime;
                    }
                }
            }

            // If the velocity of all the bodies of the island is under the
            // sleeping velocity threshold for a period of time larger than
            // the time required to become a sleeping body
            if (minSleepTime >= mTimeBeforeSleep) {

                // Put all the bodies of the island to sleep
                for (int b = 0; b < mIslands[i].getNbBodies(); b++) {
                    bodies[b].setIsSleeping(true);
                }
            }
        }
    }

    /// Update the overlapping pair
    @Override
    protected void updateOverlappingPair(BroadPhasePair pair) {
    }

    // Notify the world about a new broad-phase overlapping pair
    @Override
    protected void notifyAddedOverlappingPair(BroadPhasePair addedPair) {

        // Get the pair of body index
        BodyIndexPair indexPair = addedPair.getBodiesIndexPair();

        // Add the pair into the set of overlapping pairs (if not there yet)
        OverlappingPair newPair = new OverlappingPair(addedPair.body1, addedPair.body2, mMemoryAllocator);
        assert (newPair != null);
        //std::pair<Map<BodyIndexPair, OverlappingPair> , boolean> check =
        //        mOverlappingPairs.insert(make_pair(indexPair, newPair));
        assert (check.second);
    }

    // Notify the world about a removed broad-phase overlapping pair
    @Override
    protected void notifyRemovedOverlappingPair(BroadPhasePair removedPair) {

        // Get the pair of body index
        BodyIndexPair indexPair = removedPair.getBodiesIndexPair();

        // Remove the overlapping pair from the memory allocator
        //mOverlappingPairs.find(indexPair).second.OverlappingPair::~OverlappingPair();
        //mMemoryAllocator.release(mOverlappingPairs[indexPair], sizeof(OverlappingPair));
        mOverlappingPairs.erase(indexPair);
    }

    // Notify the world about a new narrow-phase contact
    @Override
    protected void notifyNewContact(BroadPhasePair broadPhasePair, ContactPointInfo contactInfo) {

        // Create a new contact
        ContactPoint contact = new ContactPoint(contactInfo);
        assert (contact != null);

        // Get the corresponding overlapping pair
        BodyIndexPair indexPair = broadPhasePair.getBodiesIndexPair();
        OverlappingPair overlappingPair = mOverlappingPairs.find(indexPair).second;
        assert (overlappingPair != null);

        // If it is the first contact since the pair are overlapping
        if (overlappingPair.getNbContactPoints() == 0) {

            // Trigger a callback event
            if (mEventListener != null) {
                mEventListener.beginContact(contactInfo);
            }
        }

        // Add the contact to the contact cache of the corresponding overlapping pair
        overlappingPair.addContact(contact);

        // Add the contact manifold to the world
        mContactManifolds.push_back(overlappingPair.getContactManifold());

        // Add the contact manifold into the list of contact manifolds
        // of the two bodies involved in the contact
        addContactManifoldToBody(overlappingPair.getContactManifold(), overlappingPair.mBody1,
                overlappingPair.mBody2);

        // Trigger a callback event for the new contact
        if (mEventListener != null) {
            mEventListener.newContact(contactInfo);
        }
    }

    // Constructor
    public DynamicsWorld(Vector3 gravity, float timeStep) {
        super();

        mTimer = new Timer(timeStep);
        mGravity = gravity;
        mIsGravityEnabled = true;
        mConstrainedLinearVelocities = null;
        mConstrainedAngularVelocities = null;
        mContactSolver = new ContactSolver(mMapBodyToConstrainedVelocityIndex);
        mConstraintSolver = new ConstraintSolver(mConstrainedPositions, mConstrainedOrientations, mMapBodyToConstrainedVelocityIndex);
        mNbVelocitySolverIterations = Defaults.DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS;
        mNbPositionSolverIterations = Defaults.DEFAULT_POSITION_SOLVER_NB_ITERATIONS;
        mIsSleepingEnabled = Defaults.SPLEEPING_ENABLED;
        mSplitLinearVelocities = null;
        mSplitAngularVelocities = null;
        mNbIslands = 0;
        mNbIslandsCapacity = 0;
        mIslands = null;
        mNbBodiesCapacity = 0;
        mSleepLinearVelocity = Defaults.DEFAULT_SLEEP_LINEAR_VELOCITY;
        mSleepAngularVelocity = Defaults.DEFAULT_SLEEP_ANGULAR_VELOCITY;
        mTimeBeforeSleep = Defaults.DEFAULT_TIME_BEFORE_SLEEP;
        mEventListener = null;
    }

    // Start the physics simulation
    public void start() {
        mTimer.start();
    }

    public void stop() {
        mTimer.stop();
    }

    // Set the number of iterations for the velocity constraint solver
    public void setNbIterationsVelocitySolver(int nbIterations) {
        mNbVelocitySolverIterations = nbIterations;
    }

    // Set the number of iterations for the position constraint solver
    public void setNbIterationsPositionSolver(int nbIterations) {
        mNbPositionSolverIterations = nbIterations;
    }

    // Set the position correction technique used for contacts
    public void setContactsPositionCorrectionTechnique(ContactsPositionCorrectionTechnique technique) {
        if (technique == ContactsPositionCorrectionTechnique.BAUMGARTE_CONTACTS) {
            mContactSolver.setIsSplitImpulseActive(false);
        } else {
            mContactSolver.setIsSplitImpulseActive(true);
        }
    }

    // Set the position correction technique used for joints
    public void setJointsPositionCorrectionTechnique(JointsPositionCorrectionTechnique technique) {
        if (technique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mConstraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(false);
        } else {
            mConstraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(true);
        }
    }

    // Activate or deactivate the solving of friction constraints at the center of
    // the contact manifold instead of solving them at each contact point
    public void setIsSolveFrictionAtContactManifoldCenterActive(boolean isActive) {
        mContactSolver.setIsSolveFrictionAtContactManifoldCenterActive(isActive);
    }

    // Update the overlapping pair
    protected void updateOverlappingPair(BroadPhasePair pair) {

        // Get the pair of body index
        BodyIndexPair indexPair = pair.getBodiesIndexPair();

        // Get the corresponding overlapping pair
        OverlappingPair overlappingPair = mOverlappingPairs[indexPair];

        // Update the contact cache of the overlapping pair
        overlappingPair.update();
    }

    // Return the gravity vector of the world
    public Vector3 getGravity() {
        return mGravity;
    }

    // Return if the gravity is enaled
    public boolean isGravityEnabled() {
        return mIsGravityEnabled;
    }

    // Enable/Disable the gravity
    public void setIsGratityEnabled(boolean isGravityEnabled) {
        mIsGravityEnabled = isGravityEnabled;
    }

    // Return the number of rigid bodies in the world
    public int getNbRigidBodies() {
        return mRigidBodies.size();
    }

    /// Return the number of joints in the world
    public int getNbJoints() {
        return mJoints.size();
    }

    // Return an iterator to the beginning of the bodies of the physics world
    public Set<RigidBody> getRigidBodiesBeginIterator() {
        return mRigidBodies.begin();
    }

    // Return an iterator to the end of the bodies of the physics world
    public Set<RigidBody> getRigidBodiesEndIterator() {
        return mRigidBodies.end();
    }

    // Return a reference to the contact manifolds of the world
    public List<ContactManifold> getContactManifolds() {
        return mContactManifolds;
    }

    // Return the number of contact manifolds in the world
    public int getNbContactManifolds() {
        return mContactManifolds.size();
    }

    // Return the current physics time (in seconds)
    public long getPhysicsTime() {
        return mTimer.getPhysicsTime();
    }

    // Return true if the sleeping technique is enabled
    public boolean isSleepingEnabled() {
        return mIsSleepingEnabled;
    }

    // Return the current sleep linear velocity
    public float getSleepLinearVelocity() {
        return mSleepLinearVelocity;
    }

    // Set the sleep linear velocity.
    /// When the velocity of a body becomes smaller than the sleep linear/angular
    /// velocity for a given amount of time, the body starts sleeping and does not need
    /// to be simulated anymore.
    public void setSleepLinearVelocity(float sleepLinearVelocity) {
        assert (sleepLinearVelocity >= 0.0f);
        mSleepLinearVelocity = sleepLinearVelocity;
    }

    // Return the current sleep angular velocity
    public float getSleepAngularVelocity() {
        return mSleepAngularVelocity;
    }

    // Set the sleep angular velocity.
    /// When the velocity of a body becomes smaller than the sleep linear/angular
    /// velocity for a given amount of time, the body starts sleeping and does not need
    /// to be simulated anymore.
    public void setSleepAngularVelocity(float sleepAngularVelocity) {
        assert (sleepAngularVelocity >= 0.0f);
        mSleepAngularVelocity = sleepAngularVelocity;
    }

    // Return the time a body is required to stay still before sleeping
    public float getTimeBeforeSleep() {
        return mTimeBeforeSleep;
    }

    // Set the time a body is required to stay still before sleeping
    public void setTimeBeforeSleep(float timeBeforeSleep) {
        assert (timeBeforeSleep >= 0.0f);
        mTimeBeforeSleep = timeBeforeSleep;
    }

    // Set an event listener object to receive events callbacks.
    /// If you use null as an argument, the events callbacks will be disabled.
    public void setEventListener(EventListener eventListener) {
        mEventListener = eventListener;
    }

    // Update the physics simulation
    public void update() {

        //#ifdef IS_PROFILING_ACTIVE
        // Increment the frame counter of the profiler
        Profiler.incrementFrameCounter();
        //#endif

        PROFILE("DynamicsWorld::update()");

        assert (mTimer.getIsRunning());

        // Compute the time since the last update() call and update the timer
        mTimer.update();

        // While the time accumulator is not empty
        while (mTimer.isPossibleToTakeStep()) {

            // Remove all contact manifolds
            mContactManifolds.clear();

            // Reset all the contact manifolds lists of each body
            resetContactManifoldListsOfBodies();

            // Compute the collision detection
            mCollisionDetection.computeCollisionDetection();

            // Compute the islands (separate groups of bodies with constraints between each others)
            computeIslands();

            // Integrate the velocities
            integrateRigidBodiesVelocities();

            // Reset the movement boolean variable of each body to false
            resetBodiesMovementVariable();

            // Update the timer
            mTimer.nextStep();

            // Solve the contacts and constraints
            solveContactsAndConstraints();

            // Integrate the position and orientation of each body
            integrateRigidBodiesPositions();

            // Solve the position correction for constraints
            solvePositionCorrection();

            if (mIsSleepingEnabled) {
                updateSleepingBodies();
            }

            // Update the AABBs of the bodies
            updateRigidBodiesAABB();
        }

        // Reset the external force and torque applied to the bodies
        resetBodiesForceAndTorque();

        // Compute and set the interpolation factor to all the bodies
        setInterpolationFactorToAllBodies();
    }

    // Create a rigid body into the physics world
    public RigidBody createRigidBody(Transform transform, float mass,
            Matrix3x3 inertiaTensorLocal,
            CollisionShape collisionShape) {

        // Compute the body ID
        int bodyID = computeNextAvailableBodyID();

        // Largest index cannot be used (it is used for invalid index)
        assert (bodyID < Integer.MAX_VALUE);

        // Create a collision shape for the rigid body into the world
        CollisionShape newCollisionShape = createCollisionShape(collisionShape);

        // Create the rigid body
        RigidBody rigidBody = new RigidBody(transform, mass, inertiaTensorLocal, newCollisionShape, bodyID);
        assert (rigidBody != null);

        // Add the rigid body to the physics world
        mBodies.insert(rigidBody);
        mRigidBodies.insert(rigidBody);

        // Add the rigid body to the collision detection
        mCollisionDetection.addBody(rigidBody);

        // Return the pointer to the rigid body
        return rigidBody;
    }

    // Destroy a rigid body and all the joints which it belongs
    public void destroyRigidBody(RigidBody rigidBody) {

        // Remove the body from the collision detection
        mCollisionDetection.removeBody(rigidBody);

        // Add the body ID to the list of free IDs
        mFreeBodiesIDs.push_back(rigidBody.getID());

        // Remove the collision shape from the world
        removeCollisionShape(rigidBody.getCollisionShape());

        // Destroy all the joints in which the rigid body to be destroyed is involved
        JointListElement element;
        for (element = rigidBody.mJointsList; element != null; element = element.next) {
            destroyJoint(element.joint);
        }

        // Reset the contact manifold list of the body
        rigidBody.resetContactManifoldsList(mMemoryAllocator);
        // Call the destructor of the rigid body
        //rigidBody.RigidBody::~RigidBody();

        // Remove the rigid body from the list of rigid bodies
        mBodies.erase(rigidBody);
        mRigidBodies.erase(rigidBody);

        // Free the object from the memory allocator
        mMemoryAllocator.release(rigidBody, sizeof(RigidBody));
    }

    // Create a joint between two bodies in the world and return a pointer to the new joint
    public Joint createJoint(JointInfo jointInfo) {

        Joint newJoint = null;

        // Allocate memory to create the new joint
        switch (jointInfo.type) {

            // Ball-and-Socket joint
            case BALLSOCKETJOINT: {
                //void* allocatedMemory = mMemoryAllocator.allocate(sizeof(BallAndSocketJoint));
                BallAndSocketJointInfo info = (BallAndSocketJointInfo) jointInfo;
                newJoint = new BallAndSocketJoint(info);
                break;
            }

            // Slider joint
            case SLIDERJOINT: {
                //void* allocatedMemory = mMemoryAllocator.allocate(sizeof(SliderJoint));
                SliderJointInfo info = (SliderJointInfo) jointInfo;
                newJoint = new SliderJoint(info);
                break;
            }

            // Hinge joint
            case HINGEJOINT: {
                //void* allocatedMemory = mMemoryAllocator.allocate(sizeof(HingeJoint));
                HingeJointInfo info = (HingeJointInfo) jointInfo;
                newJoint = new HingeJoint(info);
                break;
            }

            // Fixed joint
            case FIXEDJOINT: {
                //void* allocatedMemory = mMemoryAllocator.allocate(sizeof(FixedJoint));
                FixedJointInfo info = (FixedJointInfo) jointInfo;
                newJoint = new FixedJoint(info);
                break;
            }

            default: {
                assert (false);
                return null;
            }
        }

        // If the collision between the two bodies of the constraint is disabled
        if (!jointInfo.isCollisionEnabled) {

            // Add the pair of bodies in the set of body pairs that cannot collide with each other
            mCollisionDetection.addNoCollisionPair(jointInfo.body1, jointInfo.body2);
        }

        // Add the joint into the world
        mJoints.insert(newJoint);

        // Add the joint into the joint list of the bodies involved in the joint
        addJointToBody(newJoint);

        // Return the pointer to the created joint
        return newJoint;
    }

    // Destroy a joint
    public void destroyJoint(Joint joint) {

        assert (joint != null);

        // If the collision between the two bodies of the constraint was disabled
        if (!joint.isCollisionEnabled()) {

            // Remove the pair of bodies from the set of body pairs that cannot collide with each other
            mCollisionDetection.removeNoCollisionPair(joint.getBody1(), joint.getBody2());
        }

        // Wake up the two bodies of the joint
        joint.getBody1().setIsSleeping(false);
        joint.getBody2().setIsSleeping(false);

        // Remove the joint from the world
        mJoints.erase(joint);

        // Remove the joint from the joint list of the bodies involved in the joint
        joint.mBody1.removeJointFromJointsList(mMemoryAllocator, joint);
        joint.mBody2.removeJointFromJointsList(mMemoryAllocator, joint);

        int nbBytes = joint.getSizeInBytes();
        // Call the destructor of the joint
        //joint.Joint::~Joint();

        // Release the allocated memory
        mMemoryAllocator.release(joint, nbBytes);
    }

    // Add the joint to the list of joints of the two bodies involved in the joint
    public void addJointToBody(Joint joint) {

        assert (joint != null);

        // Add the joint at the beginning of the linked list of joints of the first body
        //void* allocatedMemory1 = mMemoryAllocator.allocate(sizeof(JointListElement));
        JointListElement jointListElement1 = new JointListElement(joint, joint.mBody1.mJointsList);
        joint.mBody1.mJointsList = jointListElement1;

        // Add the joint at the beginning of the linked list of joints of the second body
        //void* allocatedMemory2 = mMemoryAllocator.allocate(sizeof(JointListElement));
        JointListElement jointListElement2 = new JointListElement(joint, joint.mBody2.mJointsList);
        joint.mBody2.mJointsList = jointListElement2;
    }

    // Add a contact manifold to the linked list of contact manifolds of the two bodies involed
    // in the corresponding contact
    public void addContactManifoldToBody(ContactManifold contactManifold, CollisionBody body1, CollisionBody body2) {

        assert (contactManifold != null);

        // Add the contact manifold at the beginning of the linked
        // list of contact manifolds of the first body
        //void* allocatedMemory1 = mMemoryAllocator.allocate(sizeof(ContactManifoldListElement));
        ContactManifoldListElement listElement1 = new ContactManifoldListElement(contactManifold, body1.mContactManifoldsList);
        body1.mContactManifoldsList = listElement1;

        // Add the joint at the beginning of the linked list of joints of the second body
        //void* allocatedMemory2 = mMemoryAllocator.allocate(sizeof(ContactManifoldListElement));
        ContactManifoldListElement listElement2 = new ContactManifoldListElement(contactManifold, body2.mContactManifoldsList);
        body2.mContactManifoldsList = listElement2;
    }

    // Reset all the contact manifolds linked list of each body
    public void resetContactManifoldListsOfBodies() {

        // For each rigid body of the world
        for (Set<RigidBody> it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

            // Reset the contact manifold list of the body
            it.resetContactManifoldsList(mMemoryAllocator);
        }
    }

    // Enable/Disable the sleeping technique
    public void enableSleeping(boolean isSleepingEnabled) {
        mIsSleepingEnabled = isSleepingEnabled;

        if (!mIsSleepingEnabled) {

            // For each body of the world
            Set<RigidBody> it;
            for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

                // Wake up the rigid body
                it.setIsSleeping(false);
            }
        }
    }

}
