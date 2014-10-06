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

/**
 * Utility functions used by several classes.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Utils {

    // Encode a floating value into a integer value in order to
    // work with integer comparison in the Sweep-And-Prune algorithm
    // because it is faster. The main issue when encoding floating
    // number into integer is to keep to sorting order. This is a
    // problem for negative float number. This article describes
    // how to solve this issue : http://www.stereopsis.com/radix.html
    public static long encodeFloatIntoInteger(float number) {
        long intNumber = (long) Float.floatToIntBits(number) & 0xFFFFFFFFl;

        if ((intNumber & 0x80000000l) == 0x80000000l) {
            // If it's a negative number
            intNumber = ~intNumber & 0xFFFFFFFFl;
        } else {
            // If it's a positive number
            intNumber |= 0x80000000l;
        }

        return intNumber;
    }

}
