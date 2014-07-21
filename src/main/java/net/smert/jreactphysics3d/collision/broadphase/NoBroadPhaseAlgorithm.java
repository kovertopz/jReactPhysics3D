package net.smert.jreactphysics3d.collision.broadphase;

import java.util.Set;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.collision.CollisionDetection;
import net.smert.jreactphysics3d.collision.shapes.AABB;

/**
 * This class implements a broad-phase algorithm that does nothing. It should be use if we don't want to perform a
 * broad-phase for the collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class NoBroadPhaseAlgorithm extends BroadPhaseAlgorithm {

    /// All bodies of the world
    protected Set<CollisionBody> mBodies;

    // Constructor
    public NoBroadPhaseAlgorithm(CollisionDetection collisionDetection) {
        super(collisionDetection);
    }

    // Notify the broad-phase about a new object in the world
    @Override
    public void addObject(CollisionBody body, AABB aabb) {

        // For each body that is already in the world
        for (CollisionBody it : mBodies) {

            // Add an overlapping pair with the new body
            mPairManager.addPair(it, body);
        }

        // Add the new body into the list of bodies
        mBodies.add(body);
    }

    // Notify the broad-phase about an object that has been removed from the world
    @Override
    public void removeObject(CollisionBody body) {

        // For each body that is in the world
        for (CollisionBody it : mBodies) {

            if (it.getID() != body.getID()) {

                // Remove the overlapping pair with the new body
                mPairManager.removePair(it.getID(), body.getID());
            }
        }

        // Remove the body from the broad-phase
        mBodies.remove(body);
    }

    // Notify the broad-phase that the AABB of an object has changed
    @Override
    public void updateObject(CollisionBody body, AABB aabb) {
        // Do nothing
    }

}
