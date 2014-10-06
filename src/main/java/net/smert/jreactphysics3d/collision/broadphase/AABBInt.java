/*
 * ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/
 * Copyright (c) 2010-2013 Daniel Chappuis
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim
 *    that you wrote the original software. If you use this software in a
 *    product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * This file has been modified during the port to Java and differ from the source versions.
 */
package net.smert.jreactphysics3d.collision.broadphase;

import net.smert.jreactphysics3d.collision.shapes.AABB;

/**
 * Axis-Aligned Bounding box with integer coordinates.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class AABBInt {

    // Minimum values on the three axis
    public final long[] min = new long[3];

    // Maximum values on the three axis
    public final long[] max = new long[3];

    // Constructor that takes an AABB as input
    public AABBInt(AABB aabb) {
        min[0] = Utils.encodeFloatIntoInteger(aabb.getMin().getX());
        min[1] = Utils.encodeFloatIntoInteger(aabb.getMin().getY());
        min[2] = Utils.encodeFloatIntoInteger(aabb.getMin().getZ());

        max[0] = Utils.encodeFloatIntoInteger(aabb.getMax().getX());
        max[1] = Utils.encodeFloatIntoInteger(aabb.getMax().getY());
        max[2] = Utils.encodeFloatIntoInteger(aabb.getMax().getZ());
    }

    // Constructor that set all the axis with an minimum and maximum value
    public AABBInt(long minValue, long maxValue) {
        min[0] = minValue;
        min[1] = minValue;
        min[2] = minValue;

        max[0] = maxValue;
        max[1] = maxValue;
        max[2] = maxValue;
    }

}
