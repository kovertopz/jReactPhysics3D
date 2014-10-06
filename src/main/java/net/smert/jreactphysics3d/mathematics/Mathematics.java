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
package net.smert.jreactphysics3d.mathematics;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Mathematics {

    private Mathematics() {
    }

    // Function to test if two real numbers are (almost) equal
    // We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
    public static boolean ApproxEqual(float a, float b, float epsilon) {
        float difference = a - b;
        return (difference < epsilon && difference > -epsilon);
    }

    public static float ArcCos(float radians) {
        return (float) StrictMath.acos(radians);
    }

    public static float ArcSin(float radians) {
        return (float) StrictMath.asin(radians);
    }

    public static float ArcTan2(float a, float b) {
        return (float) StrictMath.atan2(a, b);
    }

    // Function that returns the result of the "value" clamped by
    // two others values "lowerLimit" and "upperLimit"
    public static float Clamp(float value, float lowerLimit, float upperLimit) {
        assert (lowerLimit <= upperLimit);
        return Math.min(Math.max(value, lowerLimit), upperLimit);
    }

    public static float Cos(float radians) {
        return (float) StrictMath.cos(radians);
    }

    public static float Sin(float radians) {
        return (float) StrictMath.sin(radians);
    }

    public static float Sqrt(float a) {
        return (float) StrictMath.sqrt(a);
    }

    public static float Tan(float radians) {
        return (float) StrictMath.tan(radians);
    }

}
