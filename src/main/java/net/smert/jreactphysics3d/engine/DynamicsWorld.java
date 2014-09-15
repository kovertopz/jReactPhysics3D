package net.smert.jreactphysics3d.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import net.smert.jreactphysics3d.collision.BodyIndexPair;
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
import net.smert.jreactphysics3d.mathematics.Mathematics;
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

    // True if the gravity force is on
    protected boolean isGravityEnabled;

    // True if the spleeping technique for inactive bodies is enabled
    protected boolean isSleepingEnabled;

    // Sleep angular velocity threshold
    protected float sleepAngularVelocity;

    // Sleep linear velocity threshold
    protected float sleepLinearVelocity;

    // Time (in seconds) before a body is put to sleep if its velocity
    // becomes smaller than the sleep velocity.
    protected float timeBeforeSleep;

    // Number of islands in the world
    protected int numIslands;

    // Current allocated capacity for the islands
    protected int numIslandsCapacity;

    // Number of iterations for the position solver of the Sequential Impulses technique
    protected int numPositionSolverIterations;

    // Current allocated capacity for the bodies
    protected int numRigidBodiesCapacity;

    // Number of iterations for the velocity solver of the Sequential Impulses technique
    protected int numVelocitySolverIterations;

    // Constraint solver
    protected ConstraintSolver constraintSolver;

    // Contact solver
    protected ContactSolver contactSolver;

    // Pointer to an event listener object
    protected EventListener eventListener;

    // Array with all the islands of awaken bodies
    protected Island[] islands;

    // All the contact constraints
    // TODO : Remove this variable (we will use the ones in the island now)
    protected final List<ContactManifold> contactManifolds;

    // Array of constrained rigid bodies orientation (for position error correction)
    protected final List<Quaternion> constrainedOrientations;

    // Array of constrained rigid bodies position (for position error correction)
    protected final List<Vector3> constrainedPositions;

    // Map body to their index in the constrained velocities array
    protected final Map<RigidBody, Integer> mapBodyToConstrainedVelocityIndex;

    // All the joints of the world
    protected final Set<Joint> joints;

    // All the rigid bodies of the physics world
    protected final Set<RigidBody> rigidBodies;

    // Timer of the physics engine
    protected Timer timer;

    // Gravity vector of the world
    protected Vector3 gravity;

    // Array of constrained angular velocities (state of the angular velocities
    // after solving the constraints)
    protected Vector3[] constrainedAngularVelocities;

    // Array of constrained linear velocities (state of the linear velocities
    // after solving the constraints)
    protected Vector3[] constrainedLinearVelocities;

    // Split angular velocities for the position contact solver (split impulse)
    protected Vector3[] splitAngularVelocities;

    // Split linear velocities for the position contact solver (split impulse)
    protected Vector3[] splitLinearVelocities;

    // Constructor
    public DynamicsWorld(Vector3 gravity, float timeStep) {
        super();

        isGravityEnabled = true;
        isSleepingEnabled = Defaults.SPLEEPING_ENABLED;
        sleepAngularVelocity = Defaults.DEFAULT_SLEEP_ANGULAR_VELOCITY;
        sleepLinearVelocity = Defaults.DEFAULT_SLEEP_LINEAR_VELOCITY;
        timeBeforeSleep = Defaults.DEFAULT_TIME_BEFORE_SLEEP;
        numIslands = 0;
        numIslandsCapacity = 0;
        numPositionSolverIterations = Defaults.DEFAULT_POSITION_SOLVER_NUM_ITERATIONS;
        numRigidBodiesCapacity = 0;
        numVelocitySolverIterations = Defaults.DEFAULT_VELOCITY_SOLVER_NUM_ITERATIONS;

        contactManifolds = new ArrayList<>();
        constrainedPositions = new ArrayList<>();
        constrainedOrientations = new ArrayList<>();
        mapBodyToConstrainedVelocityIndex = new HashMap<>();
        joints = new HashSet<>();
        rigidBodies = new HashSet<>();

        constraintSolver = new ConstraintSolver(constrainedPositions, constrainedOrientations, mapBodyToConstrainedVelocityIndex);
        contactSolver = new ContactSolver(mapBodyToConstrainedVelocityIndex);
        eventListener = null;
        islands = null;
        timer = new Timer(timeStep);
        this.gravity = gravity;
        constrainedLinearVelocities = null;
        constrainedAngularVelocities = null;
        splitLinearVelocities = null;
        splitAngularVelocities = null;
    }

    // Compute the islands of awake bodies.
    // An island is an isolated group of rigid bodies that have constraints (joints or contacts)
    // between each other. This method computes the islands at each time step as follows: For each
    // awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
    // (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
    // find all the bodies that are connected with it (the bodies that share joints or contacts with
    // it). Then, we create an island with this group of connected bodies.
    protected void computeIslands() {

        Profiler.StartProfilingBlock("DynamicsWorld::computeIslands()");

        int numBodies = rigidBodies.size();

        // Clear all the islands
        for (int i = 0; i < numIslands; i++) {
            // Call the island destructor
            // Release the allocated memory for the island
            islands[i] = null;
        }

        // Allocate and create the array of islands
        if (numIslandsCapacity != numBodies && numBodies > 0) {
            islands = new Island[numBodies];
            numIslandsCapacity = numBodies;
        }
        numIslands = 0;

        // Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
        for (RigidBody it : rigidBodies) {
            it.setIsAlreadyInIsland(false);
        }
        for (ContactManifold it : contactManifolds) {
            it.setIsAlreadyInIsland(false);
        }
        for (Joint it : joints) {
            it.setIsAlreadyInIsland(false);
        }

        // Create a stack (using an array) for the rigid bodies to visit during the Depth First Search
        RigidBody[] stackBodiesToVisit = new RigidBody[numBodies];

        // For each rigid body of the world
        for (RigidBody it : rigidBodies) {

            RigidBody body = it;

            // If the body has already been added to an island, we go to the next body
            if (body.isAlreadyInIsland()) {
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
            body.setIsAlreadyInIsland(true);

            // Create the new island
            islands[numIslands] = new Island(numBodies, contactManifolds.size(), joints.size());

            // While there are still some bodies to visit in the stack
            while (stackIndex > 0) {

                // Get the next body to visit from the stack
                stackIndex--;
                RigidBody bodyToVisit = stackBodiesToVisit[stackIndex];
                assert (bodyToVisit.isActive());

                // Awake the body if it is slepping
                bodyToVisit.setIsSleeping(false);

                // Add the body into the island
                islands[numIslands].addBody(bodyToVisit);

                // If the current body is not moving, we do not want to perform the DFS
                // search across that body
                if (!bodyToVisit.isMotionEnabled()) {
                    continue;
                }

                // For each contact manifold in which the current body is involded
                ContactManifoldListElement contactElement;
                for (contactElement = bodyToVisit.getContactManifoldsLists(); contactElement != null;
                        contactElement = contactElement.getNext()) {

                    ContactManifold contactManifold = contactElement.getContactManifold();

                    // Check if the current contact manifold has already been added into an island
                    if (contactManifold.isAlreadyInIsland()) {
                        continue;
                    }

                    // Add the contact manifold into the island
                    islands[numIslands].addContactManifold(contactManifold);
                    contactManifold.setIsAlreadyInIsland(true);

                    // Get the other body of the contact manifold
                    RigidBody body1 = (RigidBody) (contactManifold.getBody1());
                    RigidBody body2 = (RigidBody) (contactManifold.getBody2());
                    RigidBody otherBody = (body1.getBodyID() == bodyToVisit.getBodyID()) ? body2 : body1;

                    // Check if the other body has already been added to the island
                    if (otherBody.isAlreadyInIsland()) {
                        continue;
                    }

                    // Insert the other body into the stack of bodies to visit
                    stackBodiesToVisit[stackIndex] = otherBody;
                    stackIndex++;
                    otherBody.setIsAlreadyInIsland(true);
                }

                // For each joint in which the current body is involved
                JointListElement jointElement;
                for (jointElement = bodyToVisit.getJointsList(); jointElement != null;
                        jointElement = jointElement.next) {

                    Joint joint = jointElement.joint;

                    // Check if the current joint has already been added into an island
                    if (joint.isAlreadyInIsland()) {
                        continue;
                    }

                    // Add the joint into the island
                    islands[numIslands].addJoint(joint);
                    joint.setIsAlreadyInIsland(true);

                    // Get the other body of the contact manifold
                    RigidBody body1 = (RigidBody) (joint.getBody1());
                    RigidBody body2 = (RigidBody) (joint.getBody2());
                    RigidBody otherBody = (body1.getBodyID() == bodyToVisit.getBodyID()) ? body2 : body1;

                    // Check if the other body has already been added to the island
                    if (otherBody.isAlreadyInIsland()) {
                        continue;
                    }

                    // Insert the other body into the stack of bodies to visit
                    stackBodiesToVisit[stackIndex] = otherBody;
                    stackIndex++;
                    otherBody.setIsAlreadyInIsland(true);
                }
            }

            // Reset the isAlreadyIsland variable of the static bodies so that they
            // can also be included in the other islands
            for (int i = 0; i < islands[numIslands].getNumBodies(); i++) {

                if (!islands[numIslands].getBodies()[i].isMotionEnabled()) {
                    islands[numIslands].getBodies()[i].setIsAlreadyInIsland(false);
                }
            }

            numIslands++;
        }

        // Release the allocated memory for the stack of bodies to visit
    }

    // Initialize the bodies velocities arrays for the next simulation step.
    protected void initVelocityArrays() {

        // Allocate memory for the bodies velocity arrays
        int numBodies = rigidBodies.size();
        if (numRigidBodiesCapacity != numBodies && numBodies > 0) {
            constrainedAngularVelocities = new Vector3[numBodies];
            constrainedLinearVelocities = new Vector3[numBodies];
            numRigidBodiesCapacity = numBodies;
            splitAngularVelocities = new Vector3[numBodies];
            splitLinearVelocities = new Vector3[numBodies];
            assert (constrainedAngularVelocities != null);
            assert (constrainedLinearVelocities != null);
            assert (splitAngularVelocities != null);
            assert (splitLinearVelocities != null);
        }

        // Reset the velocities arrays
        for (int i = 0; i < numRigidBodiesCapacity; i++) {
            splitLinearVelocities[i] = new Vector3();
            splitAngularVelocities[i] = new Vector3();
        }

        // Initialize the map of body indexes in the velocity arrays
        mapBodyToConstrainedVelocityIndex.clear();
        int indexBody = 0;
        for (RigidBody it : rigidBodies) {

            // Add the body into the map
            mapBodyToConstrainedVelocityIndex.put(it, indexBody);
            indexBody++;
        }
    }

    // Integrate position and orientation of the rigid bodies.
    // The positions and orientations of the bodies are integrated using
    // the sympletic Euler time stepping scheme.
    protected void integrateRigidBodiesPositions() {

        Profiler.StartProfilingBlock("DynamicsWorld::integrateRigidBodiesPositions()");

        float dt = (float) timer.getTimeStep();

        // For each island of the world
        for (int i = 0; i < numIslands; i++) {

            RigidBody[] bodies = islands[i].getBodies();

            // For each body of the island
            for (int b = 0; b < islands[i].getNumBodies(); b++) {

                // If the body is allowed to move
                if (bodies[b].isMotionEnabled()) {

                    // Get the constrained velocity
                    int indexArray = mapBodyToConstrainedVelocityIndex.get(bodies[b]);
                    Vector3 newLinVelocity = constrainedLinearVelocities[indexArray];
                    Vector3 newAngVelocity = constrainedAngularVelocities[indexArray];

                    // Update the linear and angular velocity of the body
                    bodies[b].setLinearVelocity(newLinVelocity);
                    bodies[b].setAngularVelocity(newAngVelocity);

                    // Add the split impulse velocity from Contact Solver (only used
                    // to update the position)
                    if (contactSolver.isSplitImpulseActive()) {

                        newLinVelocity.add(splitLinearVelocities[indexArray]);
                        newAngVelocity.add(splitAngularVelocities[indexArray]);
                    }

                    // Get current position and orientation of the body
                    Vector3 currentPosition = bodies[b].getTransform().getPosition();
                    Quaternion currentOrientation = bodies[b].getTransform().getOrientation();

                    // Compute the new position of the body
                    Vector3 newPosition = new Vector3(currentPosition).add(new Vector3(newLinVelocity).multiply(dt));
                    Quaternion newOrientation = new Quaternion(currentOrientation).add(
                            new Quaternion(newAngVelocity, 0.0f).multiply(currentOrientation).multiply(0.5f * dt));

                    // Update the Transform of the body
                    Transform newTransform = new Transform(newPosition, new Quaternion(newOrientation).normalize());
                    bodies[b].setTransform(newTransform);
                }
            }
        }
    }

    // Integrate the velocities of rigid bodies.
    // This method only set the temporary velocities but does not update
    // the actual velocitiy of the bodies. The velocities updated in this method
    // might violate the constraints and will be corrected in the constraint and
    // contact solver.
    protected void integrateRigidBodiesVelocities() {

        Profiler.StartProfilingBlock("DynamicsWorld::integrateRigidBodiesVelocities()");

        // Initialize the bodies velocity arrays
        initVelocityArrays();

        float dt = (float) timer.getTimeStep();

        // For each island of the world
        for (int i = 0; i < numIslands; i++) {

            RigidBody[] bodies = islands[i].getBodies();

            // For each body of the island
            for (int b = 0; b < islands[i].getNumBodies(); b++) {

                // Insert the body into the map of constrained velocities
                int indexBody = mapBodyToConstrainedVelocityIndex.get(bodies[b]);

                assert (splitLinearVelocities[indexBody].equals(new Vector3()));
                assert (splitAngularVelocities[indexBody].equals(new Vector3()));

                // If the body is allowed to move
                if (bodies[b].isMotionEnabled()) {

                    // Integrate the external force to get the new velocity of the body
                    constrainedLinearVelocities[indexBody] = new Vector3(bodies[b].getLinearVelocity()).add(
                            new Vector3(bodies[b].getExternalForce()).multiply(dt * bodies[b].getMassInverse()));
                    constrainedAngularVelocities[indexBody] = new Vector3(bodies[b].getAngularVelocity()).add(
                            new Matrix3x3(bodies[b].getInertiaTensorInverseWorld()).multiply(dt).multiply(
                                    bodies[b].getExternalTorque(), new Vector3()));

                    // If the gravity has to be applied to this rigid body
                    if (bodies[b].isGravityEnabled() && isGravityEnabled) {

                        // Integrate the gravity force
                        constrainedLinearVelocities[indexBody].add(new Vector3(
                                gravity).multiply(dt * bodies[b].getMassInverse() * bodies[b].getMass()));
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
                    float linearDamping = Mathematics.Clamp(1.0f - dt * linDampingFactor, 0.0f, 1.0f);
                    float angularDamping = Mathematics.Clamp(1.0f - dt * angDampingFactor, 0.0f, 1.0f);
                    constrainedLinearVelocities[indexBody].multiply(Mathematics.Clamp(linearDamping, 0.0f, 1.0f));
                    constrainedAngularVelocities[indexBody].multiply(Mathematics.Clamp(angularDamping, 0.0f, 1.0f));

                    // Update the old Transform of the body
                    bodies[b].updateOldTransform();
                } else {
                    constrainedLinearVelocities[indexBody] = new Vector3(0, 0, 0);
                    constrainedAngularVelocities[indexBody] = new Vector3(0, 0, 0);
                }

                indexBody++;
            }
        }
    }

    // Reset the external force and torque applied to the bodies
    protected void resetBodiesForceAndTorque() {

        // For each body of the world
        for (RigidBody it : rigidBodies) {
            it.getExternalForce().zero();
            it.getExternalTorque().zero();
        }
    }

    // Reset the boolean movement variable of each body
    protected void resetBodiesMovementVariable() {

        // For each rigid body
        for (RigidBody it : getRigidBodies()) {

            // Set the hasMoved variable to false
            it.setHasMoved(false);
        }
    }

    // Compute and set the interpolation factor to all bodies
    protected void setInterpolationFactorToAllBodies() {

        Profiler.StartProfilingBlock("DynamicsWorld::setInterpolationFactorToAllBodies()");

        // Compute the interpolation factor
        float factor = timer.computeInterpolationFactor();
        assert (factor >= 0.0f && factor <= 1.0f);

        // Set the factor to all bodies
        for (RigidBody it : rigidBodies) {

            it.setInterpolationFactor(factor);
        }
    }

    // Solve the contacts and constraints
    protected void solveContactsAndConstraints() {

        Profiler.StartProfilingBlock("DynamicsWorld::solveContactsAndConstraints()");

        // Get the current time step
        float dt = (float) timer.getTimeStep();

        // Set the velocities arrays
        contactSolver.setSplitVelocitiesArrays(splitLinearVelocities, splitAngularVelocities);
        contactSolver.setConstrainedVelocitiesArrays(constrainedLinearVelocities, constrainedAngularVelocities);
        constraintSolver.setConstrainedVelocitiesArrays(constrainedLinearVelocities, constrainedAngularVelocities);

        // ---------- Solve velocity constraints for joints and contacts ---------- //
        // For each island of the world
        for (int islandIndex = 0; islandIndex < numIslands; islandIndex++) {

            // Check if there are contacts and constraints to solve
            boolean isConstraintsToSolve = islands[islandIndex].getNumJoints() > 0;
            boolean isContactsToSolve = islands[islandIndex].getNumContactManifolds() > 0;
            if (!isConstraintsToSolve && !isContactsToSolve) {
                continue;
            }

            // If there are contacts in the current island
            if (isContactsToSolve) {

                // Initialize the solver
                contactSolver.initializeForIsland(dt, islands[islandIndex]);

                // Warm start the contact solver
                contactSolver.warmStart();
            }

            // If there are constraints
            if (isConstraintsToSolve) {

                // Initialize the constraint solver
                constraintSolver.initializeForIsland(dt, islands[islandIndex]);
            }

            // For each iteration of the velocity solver
            for (int i = 0; i < numVelocitySolverIterations; i++) {

                // Solve the constraints
                if (isConstraintsToSolve) {
                    constraintSolver.solveVelocityConstraints(islands[islandIndex]);
                }

                // Solve the contacts
                if (isContactsToSolve) {
                    contactSolver.solve();
                }
            }

            // Cache the lambda values in order to use them in the next
            // step and cleanup the contact solver
            if (isContactsToSolve) {
                contactSolver.storeImpulses();
                contactSolver.cleanup();
            }
        }
    }

    // Solve the position error correction of the constraints
    protected void solvePositionCorrection() {

        Profiler.StartProfilingBlock("DynamicsWorld::solvePositionCorrection()");

        // Do not continue if there is no constraints
        if (joints.isEmpty()) {
            return;
        }

        // ---------- Get the position/orientation of the rigid bodies ---------- //
        // TODO : Use better memory allocation here
        constrainedPositions.clear();
        constrainedOrientations.clear();

        // For each island of the world
        for (int islandIndex = 0; islandIndex < numIslands; islandIndex++) {

            // For each body of the island
            RigidBody[] bodies = islands[islandIndex].getBodies();
            for (int b = 0; b < islands[islandIndex].getNumBodies(); b++) {

                int index = mapBodyToConstrainedVelocityIndex.get(bodies[b]);

                // Get the position/orientation of the rigid body
                Transform transform = bodies[b].getTransform();
                constrainedPositions.set(index, transform.getPosition());
                constrainedOrientations.set(index, transform.getOrientation());
            }

            // ---------- Solve the position error correction for the constraints ---------- //
            // For each iteration of the position (error correction) solver
            for (int i = 0; i < numPositionSolverIterations; i++) {

                // Solve the position constraints
                constraintSolver.solvePositionConstraints(islands[islandIndex]);
            }

            // ---------- Update the position/orientation of the rigid bodies ---------- //
            for (int b = 0; b < islands[islandIndex].getNumBodies(); b++) {

                int index = mapBodyToConstrainedVelocityIndex.get(bodies[b]);

                // Get the new position/orientation of the body
                Vector3 newPosition = constrainedPositions.get(index);
                Quaternion newOrientation = constrainedOrientations.get(index);

                // Update the Transform of the body
                Transform newTransform = new Transform(newPosition, new Quaternion(newOrientation).normalize());
                bodies[b].setTransform(newTransform);
            }
        }
    }

    // Update the AABBs of the bodies
    protected void updateRigidBodiesAABB() {

        Profiler.StartProfilingBlock("DynamicsWorld::updateRigidBodiesAABB()");

        // For each rigid body of the world
        for (RigidBody it : rigidBodies) {

            // If the body has moved
            if (it.getHasMoved()) {

                // Update the AABB of the rigid body
                it.updateAABB();
            }
        }
    }

    // Put bodies to sleep if needed.
    // For each island, if all the bodies have been almost still for a long enough period of
    // time, we put all the bodies of the island to sleep.
    protected void updateSleepingBodies() {

        Profiler.StartProfilingBlock("DynamicsWorld::updateSleepingBodies()");

        float dt = (float) timer.getTimeStep();
        float sleepLinearVelocitySquare = sleepLinearVelocity * sleepLinearVelocity;
        float sleepAngularVelocitySquare = sleepAngularVelocity * sleepAngularVelocity;

        // For each island of the world
        for (int i = 0; i < numIslands; i++) {

            float minSleepTime = Defaults.DECIMAL_LARGEST;

            // For each body of the island
            RigidBody[] bodies = islands[i].getBodies();
            for (int b = 0; b < islands[i].getNumBodies(); b++) {

                // Skip static bodies
                if (!bodies[b].isMotionEnabled()) {
                    continue;
                }

                // If the body is velocity is large enough to stay awake
                if (bodies[b].getLinearVelocity().lengthSquare() > sleepLinearVelocitySquare
                        || bodies[b].getAngularVelocity().lengthSquare() > sleepAngularVelocitySquare
                        || !bodies[b].isAllowedToSleep()) {

                    // Reset the sleep time of the body
                    bodies[b].setSleepTime(0.0f);
                    minSleepTime = 0.0f;
                } else {  // If the body velocity is bellow the sleeping velocity threshold

                    // Increase the sleep time
                    bodies[b].setSleepTime(bodies[b].getSleepTime() + dt);
                    if (bodies[b].getSleepTime() < minSleepTime) {
                        minSleepTime = bodies[b].getSleepTime();
                    }
                }
            }

            // If the velocity of all the bodies of the island is under the
            // sleeping velocity threshold for a period of time larger than
            // the time required to become a sleeping body
            if (minSleepTime >= timeBeforeSleep) {

                // Put all the bodies of the island to sleep
                for (int b = 0; b < islands[i].getNumBodies(); b++) {
                    bodies[b].setIsSleeping(true);
                }
            }
        }
    }

    // Add the joint to the list of joints of the two bodies involved in the joint
    public void addJointToBody(Joint joint) {

        assert (joint != null);

        // Add the joint at the beginning of the linked list of joints of the first body
        JointListElement jointListElement1 = new JointListElement(joint, joint.getBody1().getJointsList());
        joint.getBody1().setJointsList(jointListElement1);

        // Add the joint at the beginning of the linked list of joints of the second body
        JointListElement jointListElement2 = new JointListElement(joint, joint.getBody2().getJointsList());
        joint.getBody2().setJointsList(jointListElement2);
    }

    // Add a contact manifold to the linked list of contact manifolds of the two bodies involed
    // in the corresponding contact
    public void addContactManifoldToBody(ContactManifold contactManifold, CollisionBody body1, CollisionBody body2) {

        assert (contactManifold != null);

        // Add the contact manifold at the beginning of the linked
        // list of contact manifolds of the first body
        ContactManifoldListElement listElement1 = new ContactManifoldListElement(contactManifold, body1.getContactManifoldsLists());
        body1.setContactManifoldsLists(listElement1);

        // Add the joint at the beginning of the linked list of joints of the second body
        ContactManifoldListElement listElement2 = new ContactManifoldListElement(contactManifold, body2.getContactManifoldsLists());
        body2.setContactManifoldsLists(listElement2);
    }

    // Create a joint between two bodies in the world and return a pointer to the new joint
    public Joint createJoint(JointInfo jointInfo) {

        Joint newJoint;

        // Allocate memory to create the new joint
        switch (jointInfo.type) {

            // Ball-and-Socket joint
            case BALLSOCKETJOINT: {
                BallAndSocketJointInfo info = (BallAndSocketJointInfo) jointInfo;
                newJoint = new BallAndSocketJoint(info);
                break;
            }

            // Slider joint
            case SLIDERJOINT: {
                SliderJointInfo info = (SliderJointInfo) jointInfo;
                newJoint = new SliderJoint(info);
                break;
            }

            // Hinge joint
            case HINGEJOINT: {
                HingeJointInfo info = (HingeJointInfo) jointInfo;
                newJoint = new HingeJoint(info);
                break;
            }

            // Fixed joint
            case FIXEDJOINT: {
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
        joints.add(newJoint);

        // Add the joint into the joint list of the bodies involved in the joint
        addJointToBody(newJoint);

        // Return the pointer to the created joint
        return newJoint;
    }

    // Create a rigid body into the physics world
    public RigidBody createRigidBody(Transform transform, float mass,
            Matrix3x3 inertiaTensorLocal, CollisionShape collisionShape) {

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
        mBodies.add(rigidBody);
        rigidBodies.add(rigidBody);

        // Add the rigid body to the collision detection
        mCollisionDetection.addBody(rigidBody);

        // Return the pointer to the rigid body
        return rigidBody;
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
        joints.remove(joint);

        // Remove the joint from the joint list of the bodies involved in the joint
        joint.getBody1().removeJointFromJointsList(joint);
        joint.getBody2().removeJointFromJointsList(joint);

        // Call the destructor of the joint
        // Release the allocated memory
    }

    // Destroy a rigid body and all the joints which it belongs
    public void destroyRigidBody(RigidBody rigidBody) {

        // Remove the body from the collision detection
        mCollisionDetection.removeBody(rigidBody);

        // Add the body ID to the list of free IDs
        mFreeBodiesIDs.add(rigidBody.getBodyID());

        // Remove the collision shape from the world
        removeCollisionShape(rigidBody.getCollisionShape());

        // Destroy all the joints in which the rigid body to be destroyed is involved
        JointListElement element;
        for (element = rigidBody.getJointsList(); element != null; element = element.next) {
            destroyJoint(element.joint);
        }

        // Reset the contact manifold list of the body
        rigidBody.resetContactManifoldsList();
        // Call the destructor of the rigid body

        // Remove the rigid body from the list of rigid bodies
        mBodies.remove(rigidBody);
        rigidBodies.remove(rigidBody);

        // Free the object from the memory allocator
    }

    // Return the current physics time (in seconds)
    public float getPhysicsTime() {
        return timer.getPhysicsTime();
    }

    // Return the current sleep angular velocity
    public float getSleepAngularVelocity() {
        return sleepAngularVelocity;
    }

    // Set the sleep angular velocity.
    // When the velocity of a body becomes smaller than the sleep linear/angular
    // velocity for a given amount of time, the body starts sleeping and does not need
    // to be simulated anymore.
    public void setSleepAngularVelocity(float sleepAngularVelocity) {
        assert (sleepAngularVelocity >= 0.0f);
        this.sleepAngularVelocity = sleepAngularVelocity;
    }

    // Return the current sleep linear velocity
    public float getSleepLinearVelocity() {
        return sleepLinearVelocity;
    }

    // Set the sleep linear velocity.
    // When the velocity of a body becomes smaller than the sleep linear/angular
    // velocity for a given amount of time, the body starts sleeping and does not need
    // to be simulated anymore.
    public void setSleepLinearVelocity(float sleepLinearVelocity) {
        assert (sleepLinearVelocity >= 0.0f);
        this.sleepLinearVelocity = sleepLinearVelocity;
    }

    // Return the time a body is required to stay still before sleeping
    public float getTimeBeforeSleep() {
        return timeBeforeSleep;
    }

    // Set the time a body is required to stay still before sleeping
    public void setTimeBeforeSleep(float timeBeforeSleep) {
        assert (timeBeforeSleep >= 0.0f);
        this.timeBeforeSleep = timeBeforeSleep;
    }

    // Return the number of contact manifolds in the world
    public int getNumContactManifolds() {
        return contactManifolds.size();
    }

    // Return the number of joints in the world
    public int getNumJoints() {
        return joints.size();
    }

    // Return the number of rigid bodies in the world
    public int getNumRigidBodies() {
        return rigidBodies.size();
    }

    // Return a reference to the contact manifolds of the world
    public List<ContactManifold> getContactManifolds() {
        return contactManifolds;
    }

    // Return an iterator to the beginning of the bodies of the physics world
    public Set<RigidBody> getRigidBodies() {
        return rigidBodies;
    }

    // Return the gravity vector of the world
    public Vector3 getGravity() {
        return gravity;
    }

    // Return if the gravity is enaled
    public boolean isGravityEnabled() {
        return isGravityEnabled;
    }

    // Enable/Disable the gravity
    public void setIsGratityEnabled(boolean isGravityEnabled) {
        this.isGravityEnabled = isGravityEnabled;
    }

    // Return true if the sleeping technique is enabled
    public boolean isSleepingEnabled() {
        return isSleepingEnabled;
    }

    // Enable/Disable the sleeping technique
    public void setIsSleepingEnabled(boolean isSleepingEnabled) {
        this.isSleepingEnabled = isSleepingEnabled;

        if (!this.isSleepingEnabled) {

            // For each body of the world
            for (RigidBody it : rigidBodies) {

                // Wake up the rigid body
                it.setIsSleeping(false);
            }
        }
    }

    // Reset all the contact manifolds linked list of each body
    public void resetContactManifoldListsOfBodies() {

        // For each rigid body of the world
        for (RigidBody it : rigidBodies) {

            // Reset the contact manifold list of the body
            it.resetContactManifoldsList();
        }
    }

    // Set the position correction technique used for contacts
    public void setContactsPositionCorrectionTechnique(ContactsPositionCorrectionTechnique technique) {
        if (technique == ContactsPositionCorrectionTechnique.BAUMGARTE_CONTACTS) {
            contactSolver.setIsSplitImpulseActive(false);
        } else {
            contactSolver.setIsSplitImpulseActive(true);
        }
    }

    // Set an event listener object to receive events callbacks.
    // If you use null as an argument, the events callbacks will be disabled.
    public void setEventListener(EventListener eventListener) {
        this.eventListener = eventListener;
    }

    // Set the position correction technique used for joints
    public void setJointsPositionCorrectionTechnique(JointsPositionCorrectionTechnique technique) {
        // TODO: Notify upstream of bug with missing method.
        if (technique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            constraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(false);
        } else {
            constraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(true);
        }
    }

    // Activate or deactivate the solving of friction constraints at the center of
    // the contact manifold instead of solving them at each contact point
    public void setIsSolveFrictionAtContactManifoldCenterActive(boolean isActive) {
        contactSolver.setIsSolveFrictionAtContactManifoldCenterActive(isActive);
    }

    // Set the number of iterations for the position constraint solver
    public void setNumIterationsPositionSolver(int numIterations) {
        numPositionSolverIterations = numIterations;
    }

    // Set the number of iterations for the velocity constraint solver
    public void setNumIterationsVelocitySolver(int numIterations) {
        numVelocitySolverIterations = numIterations;
    }

    // Start the physics simulation
    public void start() {
        timer.start();
    }

    public void stop() {
        timer.stop();
    }

    // Update the physics simulation
    public void update() {

        // Increment the frame counter of the profiler
        Profiler.IncrementFrameCounter();

        Profiler.StartProfilingBlock("DynamicsWorld::update()");

        assert (timer.getIsRunning());

        // Compute the time since the last update() call and update the timer
        timer.update();

        // While the time accumulator is not empty
        while (timer.isPossibleToTakeStep()) {

            // Remove all contact manifolds
            contactManifolds.clear();

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
            timer.nextStep();

            // Solve the contacts and constraints
            solveContactsAndConstraints();

            // Integrate the position and orientation of each body
            integrateRigidBodiesPositions();

            // Solve the position correction for constraints
            solvePositionCorrection();

            if (isSleepingEnabled) {
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

    // Notify the world about a new broad-phase overlapping pair
    @Override
    public void notifyAddedOverlappingPair(BroadPhasePair addedPair) {

        // Get the pair of body index
        BodyIndexPair indexPair = addedPair.newBodiesIndexPair();

        // Add the pair into the set of overlapping pairs (if not there yet)
        OverlappingPair newPair = new OverlappingPair(addedPair.getBody1(), addedPair.getBody2());
        assert (newPair != null);

        OverlappingPair check = mOverlappingPairs.put(indexPair, newPair);

        assert (check == null);
    }

    // Notify the world about a new narrow-phase contact
    @Override
    public void notifyNewContact(BroadPhasePair broadPhasePair, ContactPointInfo contactInfo) {

        // Create a new contact
        ContactPoint contact = new ContactPoint(contactInfo);
        assert (contact != null);

        // Get the corresponding overlapping pair
        BodyIndexPair indexPair = broadPhasePair.newBodiesIndexPair();
        OverlappingPair overlappingPair = mOverlappingPairs.get(indexPair);
        assert (overlappingPair != null);

        // If it is the first contact since the pair are overlapping
        if (overlappingPair.getNumContactPoints() == 0) {

            // Trigger a callback event
            if (eventListener != null) {
                eventListener.beginContact(contactInfo);
            }
        }

        // Add the contact to the contact cache of the corresponding overlapping pair
        overlappingPair.addContact(contact);

        // Add the contact manifold to the world
        contactManifolds.add(overlappingPair.getContactManifold());

        // Add the contact manifold into the list of contact manifolds
        // of the two bodies involved in the contact
        addContactManifoldToBody(overlappingPair.getContactManifold(), overlappingPair.getBody1(),
                overlappingPair.getBody2());

        // Trigger a callback event for the new contact
        if (eventListener != null) {
            eventListener.newContact(contactInfo);
        }
    }

    // Notify the world about a removed broad-phase overlapping pair
    @Override
    public void notifyRemovedOverlappingPair(BroadPhasePair removedPair) {

        // Get the pair of body index
        BodyIndexPair indexPair = removedPair.newBodiesIndexPair();

        // Remove the overlapping pair from the memory allocator
        mOverlappingPairs.remove(indexPair);
    }

    @Override
    // Update the overlapping pair
    public void updateOverlappingPair(BroadPhasePair pair) {

        // Get the pair of body index
        BodyIndexPair indexPair = pair.newBodiesIndexPair();

        // Get the corresponding overlapping pair
        OverlappingPair overlappingPair = mOverlappingPairs.get(indexPair);

        // Update the contact cache of the overlapping pair
        overlappingPair.update();
    }

}
