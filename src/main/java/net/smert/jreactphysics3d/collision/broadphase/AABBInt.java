package net.smert.jreactphysics3d.collision.broadphase;

import net.smert.jreactphysics3d.collision.shapes.AABB;

/**
 * Axis-Aligned Bounding box with integer coordinates.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class AABBInt {

    /// Minimum values on the three axis
    public int[] min = new int[3];

    /// Maximum values on the three axis
    public int[] max = new int[3];

    // Constructor that takes an AABB as input
    public AABBInt(AABB aabb) {
        min[0] = encodeFloatIntoInteger(aabb.getMin().x);
        min[1] = encodeFloatIntoInteger(aabb.getMin().y);
        min[2] = encodeFloatIntoInteger(aabb.getMin().z);

        max[0] = encodeFloatIntoInteger(aabb.getMax().x);
        max[1] = encodeFloatIntoInteger(aabb.getMax().y);
        max[2] = encodeFloatIntoInteger(aabb.getMax().z);
    }

    // Constructor that set all the axis with an minimum and maximum value
    public AABBInt(int minValue, int maxValue) {
        min[0] = minValue;
        min[1] = minValue;
        min[2] = minValue;

        max[0] = maxValue;
        max[1] = maxValue;
        max[2] = maxValue;
    }

}
