package net.smert.jreactphysics3d.collision.shapes;

import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a bounding volume of type "Axis Aligned Bounding Box". It's a box where all the edges are
 * always aligned with the world coordinate system. The AABB is defined by the minimum and maximum world coordinates of
 * the three axis.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class AABB {

    // Minimum world coordinates of the AABB on the x,y and z axis
    private Vector3 mMinCoordinates;

    // Maximum world coordinates of the AABB on the x,y and z axis
    private Vector3 mMaxCoordinates;

    // Constructor
    public AABB() {
        mMinCoordinates = new Vector3();
        mMaxCoordinates = new Vector3();
    }

    // Constructor
    public AABB(Vector3 minCoordinates, Vector3 maxCoordinates) {
        mMinCoordinates = minCoordinates;
        mMaxCoordinates = maxCoordinates;
    }

    // Return the center point of the AABB in world coordinates
    public Vector3 getCenter() {
        return new Vector3(mMinCoordinates).add(mMaxCoordinates).multiply(0.5f);
    }

    // Return the minimum coordinates of the AABB
    public Vector3 getMin() {
        return mMinCoordinates;
    }

    // Set the minimum coordinates of the AABB
    public void setMin(Vector3 min) {
        mMinCoordinates = min;
    }

    // Return the maximum coordinates of the AABB
    public Vector3 getMax() {
        return mMaxCoordinates;
    }

    // Set the maximum coordinates of the AABB
    public void setMax(Vector3 max) {
        mMaxCoordinates = max;
    }

    // Return true if the current AABB is overlapping with the AABB in argument.
    // Two AABBs overlap if they overlap in the three x, y and z axis at the same time
    public boolean testCollision(AABB aabb) {
        if (mMaxCoordinates.getX() < aabb.mMinCoordinates.getX() || aabb.mMaxCoordinates.getX() < mMinCoordinates.getX()) {
            return false;
        }
        if (mMaxCoordinates.getY() < aabb.mMinCoordinates.getY() || aabb.mMaxCoordinates.getY() < mMinCoordinates.getY()) {
            return false;
        }
        return mMaxCoordinates.getZ() >= aabb.mMinCoordinates.getZ() && aabb.mMaxCoordinates.getZ() >= mMinCoordinates.getZ();
    }

}
