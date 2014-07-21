package net.smert.jreactphysics3d.collision.broadphase;

import net.smert.jreactphysics3d.body.CollisionBody;

/**
 * This structure represents an AABB in the Sweep-And-Prune algorithm
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BoxAABB {

    /// Index of the 3 minimum end-points of the AABB over the x,y,z axis
    public int[] min = new int[3];

    /// Index of the 3 maximum end-points of the AABB over the x,y,z axis
    public int[] max = new int[3];

    /// Body that corresponds to the owner of the AABB
    public CollisionBody body;

}
