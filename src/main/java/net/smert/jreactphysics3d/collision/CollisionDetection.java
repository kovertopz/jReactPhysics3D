package net.smert.jreactphysics3d.collision;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.collision.broadphase.BodyPair;
import net.smert.jreactphysics3d.collision.broadphase.BroadPhaseAlgorithm;
import net.smert.jreactphysics3d.collision.broadphase.NoBroadPhaseAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.GJK.GJKAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.NarrowPhaseAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.SphereVsSphereAlgorithm;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.collision.shapes.CollisionShapeType;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
import net.smert.jreactphysics3d.engine.CollisionWorld;
import net.smert.jreactphysics3d.engine.Profiler;

/**
 * This class computes the collision detection algorithms. We first perform a broad-phase algorithm to know which pairs
 * of bodies can collide and then we run a narrow-phase algorithm to compute the collision contacts between bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CollisionDetection {

    // Broad-phase algorithm
    private final BroadPhaseAlgorithm broadPhaseAlgorithm;

    // Pointer to the physics world
    private final CollisionWorld world;

    // Narrow-phase GJK algorithm
    private final GJKAlgorithm narrowPhaseGJKAlgorithm;

    // Broad-phase overlapping pairs
    private final Map<BodyIndexPair, BroadPhasePair> overlappingPairs;

    // Set of pair of bodies that cannot collide between each other
    private final Set<BodyIndexPair> noCollisionPairs;

    // Narrow-phase Sphere vs Sphere algorithm
    private final SphereVsSphereAlgorithm narrowPhaseSphereVsSphereAlgorithm;

    // Constructor
    public CollisionDetection(CollisionWorld world) {
        assert (world != null);
        broadPhaseAlgorithm = new NoBroadPhaseAlgorithm(this);
        this.world = world;
        narrowPhaseGJKAlgorithm = new GJKAlgorithm();
        overlappingPairs = new HashMap<>();
        noCollisionPairs = new HashSet<>();
        narrowPhaseSphereVsSphereAlgorithm = new SphereVsSphereAlgorithm();
    }

    // Compute the broad-phase collision detection
    private void computeBroadPhase() {

        Profiler.startProfilingBlock("CollisionDetection::computeBroadPhase()");

        // Notify the broad-phase algorithm about the bodies that have moved since last frame
        for (CollisionBody it : world.getBodies()) {

            // If the body has moved
            if (it.getHasMoved()) {

                // Notify the broad-phase that the body has moved
                broadPhaseAlgorithm.updateObject(it, it.getAABB());
            }
        }
    }

    // Compute the narrow-phase collision detection
    private void computeNarrowPhase() {

        Profiler.startProfilingBlock("CollisionDetection::computeNarrowPhase()");

        for (Map.Entry pairs : overlappingPairs.entrySet()) {
            ContactPointInfo contactInfo = new ContactPointInfo();

            BroadPhasePair pair = (BroadPhasePair) pairs.getValue();
            assert (pair != null);

            CollisionBody body1 = pair.getBody1();
            CollisionBody body2 = pair.getBody2();

            // Update the contact cache of the overlapping pair
            world.updateOverlappingPair(pair);

            // Check if the two bodies are allowed to collide, otherwise, we do not test for collision
            if (noCollisionPairs.contains(pair.getBodiesIndexPair()) == true) {
                continue;
            }

            // Check if the two bodies are sleeping, if so, we do no test collision between them
            if (body1.isSleeping() && body2.isSleeping()) {
                continue;
            }

            // Select the narrow phase algorithm to use according to the two collision shapes
            NarrowPhaseAlgorithm narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(
                    body1.getCollisionShape(), body2.getCollisionShape());

            // Notify the narrow-phase algorithm about the overlapping pair we are going to test
            narrowPhaseAlgorithm.setCurrentOverlappingPair(pair);

            // Use the narrow-phase collision detection algorithm to check
            // if there really is a collision
            if (narrowPhaseAlgorithm.testCollision(
                    body1.getCollisionShape(), body1.getTransform(),
                    body2.getCollisionShape(), body2.getTransform(),
                    contactInfo)) {
                assert (contactInfo != null);

                // Set the bodies of the contact
                contactInfo.setBody1((RigidBody) body1);
                contactInfo.setBody2((RigidBody) body2);
                assert (contactInfo.getBody1() != null);
                assert (contactInfo.getBody2() != null);

                // Notify the world about the new narrow-phase contact
                world.notifyNewContact(pair, contactInfo);

                // Delete and remove the contact info from the memory allocator
            }
        }
    }

    // Select the narrow-phase collision algorithm to use given two collision shapes
    private NarrowPhaseAlgorithm selectNarrowPhaseAlgorithm(CollisionShape collisionShape1, CollisionShape collisionShape2) {

        // Sphere vs Sphere algorithm
        if (collisionShape1.getType() == CollisionShapeType.SPHERE && collisionShape2.getType() == CollisionShapeType.SPHERE) {
            return narrowPhaseSphereVsSphereAlgorithm;
        } else {    // GJK algorithm
            return narrowPhaseGJKAlgorithm;
        }
    }

    // Add a body to the collision detection
    public void addBody(CollisionBody body) {

        // Add the body to the broad-phase
        broadPhaseAlgorithm.addObject(body, body.getAABB());
    }

    // Remove a body from the collision detection
    public void removeBody(CollisionBody body) {

        // Remove the body from the broad-phase
        broadPhaseAlgorithm.removeObject(body);
    }

    // Add a pair of bodies that cannot collide with each other
    public void addNoCollisionPair(CollisionBody body1, CollisionBody body2) {
        noCollisionPairs.add(BroadPhasePair.ComputeBodiesIndexPair(body1, body2));
    }

    // Remove a pair of bodies that cannot collide with each other
    public void removeNoCollisionPair(CollisionBody body1, CollisionBody body2) {
        noCollisionPairs.remove(BroadPhasePair.ComputeBodiesIndexPair(body1, body2));
    }

    // Allow the broadphase to notify the collision detection about an overlapping pair.
    // This method is called by a broad-phase collision detection algorithm
    public void broadPhaseNotifyAddedOverlappingPair(BodyPair addedPair) {

        // Get the pair of body index
        BodyIndexPair indexPair = addedPair.getBodiesIndexPair();

        // Create the corresponding broad-phase pair object
        BroadPhasePair broadPhasePair = new BroadPhasePair(addedPair.getBody1(), addedPair.getBody2());
        assert (broadPhasePair != null);

        // Add the pair into the set of overlapping pairs (if not there yet)
        BroadPhasePair check = overlappingPairs.put(indexPair, broadPhasePair);

        assert (check == null);

        // Notify the world about the new broad-phase overlapping pair
        world.notifyAddedOverlappingPair(broadPhasePair);
    }

    // Allow the broadphase to notify the collision detection about a removed overlapping pair
    public void broadPhaseNotifyRemovedOverlappingPair(BodyPair removedPair) {

        // Get the pair of body index
        BodyIndexPair indexPair = removedPair.getBodiesIndexPair();

        // Get the broad-phase pair
        BroadPhasePair broadPhasePair = overlappingPairs.get(indexPair);
        assert (broadPhasePair != null);

        // Notify the world about the removed broad-phase pair
        world.notifyRemovedOverlappingPair(broadPhasePair);

        // Remove the overlapping pair from the memory allocator
        overlappingPairs.remove(indexPair);
    }

    // Compute the collision detection
    public void computeCollisionDetection() {

        Profiler.startProfilingBlock("CollisionDetection::computeCollisionDetection()");

        // Compute the broad-phase collision detection
        computeBroadPhase();

        // Compute the narrow-phase collision detection
        computeNarrowPhase();
    }

}
