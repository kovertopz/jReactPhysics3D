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

import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class represents a 3D vector.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Vector3 {

    // Component x
    float x;

    // Component y
    float y;

    // Component z
    float z;

    // Constructor
    public Vector3() {
        zero();
    }

    // Constructor with arguments
    public Vector3(float x, float y, float z) {
        set(x, y, z);
    }

    // Copy-constructor
    public Vector3(Vector3 vector) {
        set(vector);
    }

    // Return true if the vector is unit and false otherwise
    public boolean isUnit() {
        return Mathematics.ApproxEqual(lengthSquare(), 1.0f, Defaults.MACHINE_EPSILON);
    }

    // Return true if the vector is the zero vector
    public boolean isZero() {
        return Mathematics.ApproxEqual(lengthSquare(), 0.0f, Defaults.MACHINE_EPSILON);
    }

    // Scalar product of two vectors (public)
    public float dot(Vector3 vector) {
        return x * vector.x + y * vector.y + z * vector.z;
    }

    // Overloaded operator for value access
    public float get(int index) {
        if (index == 0) {
            return x;
        } else if (index == 1) {
            return y;
        } else if (index == 2) {
            return z;
        }
        throw new IllegalArgumentException("Unknown index: " + index);
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
        return z;
    }

    // Return the length of the vector
    public float length() {
        return Mathematics.Sqrt(x * x + y * y + z * z);
    }

    // Return the square of the length of the vector
    public float lengthSquare() {
        return x * x + y * y + z * z;
    }

    // Return the axis with the maximal value
    public int getMaxAxis() {
        return (x < y ? (y < z ? 2 : 1) : (x < z ? 2 : 0));
    }

    // Return the axis with the minimal value
    public int getMinAxis() {
        return (x < y ? (x < z ? 0 : 2) : (y < z ? 1 : 2));
    }

    // Return the corresponding absolute value vector
    public Vector3 abs() {
        x = Math.abs(x);
        y = Math.abs(y);
        z = Math.abs(z);
        return this;
    }

    // Overloaded operator for addition with assignment
    public Vector3 add(Vector3 vector) {
        x += vector.x;
        y += vector.y;
        z += vector.z;
        return this;
    }

    // Cross product of two vectors (public)
    public Vector3 cross(Vector3 vector) {
        return set(
                y * vector.z - z * vector.y,
                z * vector.x - x * vector.z,
                x * vector.y - y * vector.x);
    }

    // Overloaded operator for division by a number with assignment
    public Vector3 divide(float number) {
        assert (number > Defaults.MACHINE_EPSILON);
        x /= number;
        y /= number;
        z /= number;
        return this;
    }

    // Overloaded operator for the negative of a vector
    public Vector3 invert() {
        x = -x;
        y = -y;
        z = -z;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public Vector3 multiply(float number) {
        x *= number;
        y *= number;
        z *= number;
        return this;
    }

    // Normalize the vector
    public Vector3 normalize() {
        float len = length();
        assert (len > Defaults.MACHINE_EPSILON);
        float lenInv = 1.0f / len;
        x *= lenInv;
        y *= lenInv;
        z *= lenInv;
        return this;
    }

    // Set all the values of the vector
    public final Vector3 set(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    // Assignment operator
    public final Vector3 set(Vector3 vector) {
        x = vector.x;
        y = vector.y;
        z = vector.z;
        return this;
    }

    // Return one unit orthogonal vector of the current vector
    public Vector3 setUnitOrthogonal() {

        float len, lenInv;

        // Get the minimum element of the vector
        Vector3 abs = new Vector3(this).abs();
        int minElement = abs.getMinAxis();

        if (minElement == 0) {
            len = Mathematics.Sqrt(y * y + z * z);
            lenInv = 1.0f / len;
            set(0.0f, -z, y).multiply(lenInv);
        } else if (minElement == 1) {
            len = Mathematics.Sqrt(x * x + z * z);
            lenInv = 1.0f / len;
            set(-z, 0.0f, x).multiply(lenInv);
        } else {
            len = Mathematics.Sqrt(x * x + y * y);
            lenInv = 1.0f / len;
            set(-y, x, 0.0f).multiply(lenInv);
        }

        return this;
    }

    public Vector3 setX(float x) {
        this.x = x;
        return this;
    }

    public Vector3 setY(float y) {
        this.y = y;
        return this;
    }

    public Vector3 setZ(float z) {
        this.z = z;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Vector3 subtract(Vector3 vector) {
        x -= vector.x;
        y -= vector.y;
        z -= vector.z;
        return this;
    }

    // Set the vector to zero
    public final Vector3 zero() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        return this;
    }

    public static void Lerp(Vector3 oldVector, Vector3 newVector, float t, Vector3 vectorOut) {
        assert (t >= 0.0f && t <= 1.0f);
        vectorOut.set(oldVector).multiply(1.0f - t).add(new Vector3(newVector).multiply(t));
    }

    @Override
    public int hashCode() {
        int hash = 5;
        hash = 11 * hash + Float.floatToIntBits(this.x);
        hash = 11 * hash + Float.floatToIntBits(this.y);
        hash = 11 * hash + Float.floatToIntBits(this.z);
        return hash;
    }

    @Override
    public boolean equals(Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Vector3 other = (Vector3) obj;
        if (Float.floatToIntBits(this.x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(this.y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        return Float.floatToIntBits(this.z) == Float.floatToIntBits(other.z);
    }

    @Override
    public String toString() {
        return "(x= " + x + ", y= " + y + ", z= " + z + ")";
    }

}
