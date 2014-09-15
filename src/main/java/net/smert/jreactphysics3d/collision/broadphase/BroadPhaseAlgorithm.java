package net.smert.jreactphysics3d.collision.broadphase;

import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.collision.CollisionDetection;
import net.smert.jreactphysics3d.collision.shapes.AABB;

/**
 * This class is an abstract class that represents an algorithm used to perform the broad-phase of a collision
 * detection. The goal of the broad-phase algorithm is to compute the pair of bodies that can collide. But it's
 * important to understand that the broad-phase doesn't compute only body pairs that can collide but could also pairs of
 * body that doesn't collide but are very close. The goal of the broad-phase is to remove pairs of body that cannot
 * collide in order to avoid to much bodies to be tested in the narrow-phase.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class BroadPhaseAlgorithm {

    // Reference to the collision detection object
    protected CollisionDetection collisionDetection;

    // Pair manager containing the overlapping pairs
    protected PairManager pairManager;

    // Constructor
    public BroadPhaseAlgorithm(CollisionDetection collisionDetection) {
        this.collisionDetection = collisionDetection;
        pairManager = new PairManager(collisionDetection);
    }

    // Notify the broad-phase about a new object in the world
    public abstract void addObject(CollisionBody body, AABB aabb);

    // Notify the broad-phase about an object that has been removed from the world
    public abstract void removeObject(CollisionBody body);

    // Notify the broad-phase that the AABB of an object has changed
    public abstract void updateObject(CollisionBody body, AABB aabb);

}
