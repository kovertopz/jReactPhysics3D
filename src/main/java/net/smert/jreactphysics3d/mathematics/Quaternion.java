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
 * This class represents a quaternion. We use the notation : q = (x*i, y*j, z*k, w) to represent a quaternion.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Quaternion {

    // Component x
    float x;

    // Component y
    float y;

    // Component z
    float z;

    // Component w
    float w;

    // Constructor
    public Quaternion() {
        zero();
    }

    // Constructor with arguments
    public Quaternion(float x, float y, float z, float w) {
        set(x, y, z, w);
    }

    // Create a unit quaternion from a rotation matrix
    public Quaternion(Matrix3x3 matrix) {
        fromMatrix(matrix);
    }

    // Copy-constructor
    public Quaternion(Quaternion quaternion) {
        set(quaternion);
    }

    // Constructor with the component w and the vector v=(x y z)
    public Quaternion(Vector3 vector, float w) {
        set(vector, w);
    }

    // Scalar product between two quaternions
    public float dot(Quaternion quaternion) {
        return x * quaternion.x + y * quaternion.y + z * quaternion.z + w * quaternion.w;
    }

    public float getW() {
        return w;
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

    // Return the length of the quaternion (public )
    public float length() {
        return Mathematics.Sqrt(x * x + y * y + z * z + w * w);
    }

    // Return the square of the length of the quaternion
    public float lengthSquare() {
        return x * x + y * y + z * z + w * w;
    }

    // Overloaded operator for addition with assignment
    public Quaternion add(Quaternion quaternion) {
        x += quaternion.x;
        y += quaternion.y;
        z += quaternion.z;
        w += quaternion.w;
        return this;
    }

    // Return the conjugate of the quaternion (public )
    public Quaternion conjugate() {
        x = -x;
        y = -y;
        z = -z;
        return this;
    }

    public final Quaternion fromMatrix(Matrix3x3 matrix) {

        // Get the trace of the matrix
        float r, s, trace = matrix.getTrace();

        if (trace < 0.0f) {
            if (matrix.m11 > matrix.m00) {
                if (matrix.m22 > matrix.m11) {
                    r = Mathematics.Sqrt(matrix.m22 - matrix.m00 - matrix.m11 + 1.0f);
                    s = 0.5f / r;

                    // Compute the quaternion
                    x = (matrix.m20 + matrix.m02) * s;
                    y = (matrix.m12 + matrix.m21) * s;
                    z = 0.5f * r;
                    w = (matrix.m10 - matrix.m01) * s;
                } else {
                    r = Mathematics.Sqrt(matrix.m11 - matrix.m22 - matrix.m00 + 1.0f);
                    s = 0.5f / r;

                    // Compute the quaternion
                    x = (matrix.m01 + matrix.m10) * s;
                    y = 0.5f * r;
                    z = (matrix.m12 + matrix.m21) * s;
                    w = (matrix.m02 - matrix.m20) * s;
                }
            } else if (matrix.m22 > matrix.m00) {
                r = Mathematics.Sqrt(matrix.m22 - matrix.m00 - matrix.m11 + 1.0f);
                s = 0.5f / r;

                // Compute the quaternion
                x = (matrix.m20 + matrix.m02) * s;
                y = (matrix.m12 + matrix.m21) * s;
                z = 0.5f * r;
                w = (matrix.m10 - matrix.m01) * s;
            } else {
                r = Mathematics.Sqrt(matrix.m00 - matrix.m11 - matrix.m22 + 1.0f);
                s = 0.5f / r;

                // Compute the quaternion
                x = 0.5f * r;
                y = (matrix.m01 + matrix.m10) * s;
                z = (matrix.m20 - matrix.m02) * s;
                w = (matrix.m21 - matrix.m12) * s;
            }
        } else {
            r = Mathematics.Sqrt(trace + 1.0f);
            s = 0.5f / r;

            // Compute the quaternion
            x = (matrix.m21 - matrix.m12) * s;
            y = (matrix.m02 - matrix.m20) * s;
            z = (matrix.m10 - matrix.m01) * s;
            w = 0.5f * r;
        }

        return this;
    }

    // Set to the identity quaternion
    public Quaternion identity() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 1.0f;
        return this;
    }

    // Inverse the quaternion
    public Quaternion inverse() {

        // Get the square length of the quaternion
        float lenSq = lengthSquare();
        assert (lenSq > Defaults.MACHINE_EPSILON);

        // Compute and return the inverse quaternion
        x /= -lenSq;
        y /= -lenSq;
        z /= -lenSq;
        w /= lenSq;
        return this;
    }

    // Overloaded operator for the multiplication with a constant
    public Quaternion multiply(float number) {
        x *= number;
        y *= number;
        z *= number;
        w *= number;
        return this;
    }

    // Overloaded operator for the multiplication of two quaternions
    public Quaternion multiply(Quaternion quaternion) {
        Vector3 q1V = new Vector3();
        getVectorV(q1V);
        Vector3 q2V = new Vector3();
        quaternion.getVectorV(q2V);
        Vector3 newVector = new Vector3(q2V).multiply(w)
                .add(new Vector3(q1V).multiply(quaternion.w))
                .add(new Vector3(q1V).cross(q2V));
        return set(
                newVector.getX(), newVector.getY(), newVector.getZ(),
                w * quaternion.w - q1V.dot(q2V));
    }

    // Normalize the quaternion
    public Quaternion normalize() {
        float len = length();
        assert (len > Defaults.MACHINE_EPSILON);
        float lenInv = 1.0f / len;
        x *= lenInv;
        y *= lenInv;
        z *= lenInv;
        w *= lenInv;
        return this;
    }

    // Set all the values
    public final Quaternion set(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }

    public final Quaternion set(Quaternion quaternion) {
        x = quaternion.x;
        y = quaternion.y;
        z = quaternion.z;
        w = quaternion.w;
        return this;
    }

    public final Quaternion set(Vector3 vector, float w) {
        x = vector.x;
        y = vector.y;
        z = vector.z;
        this.w = w;
        return this;
    }

    public Quaternion setW(float w) {
        this.w = w;
        return this;
    }

    public Quaternion setX(float x) {
        this.x = x;
        return this;
    }

    public Quaternion setY(float y) {
        this.y = y;
        return this;
    }

    public Quaternion setZ(float z) {
        this.z = z;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Quaternion subtract(Quaternion quaternion) {
        x -= quaternion.x;
        y -= quaternion.y;
        z -= quaternion.z;
        w -= quaternion.w;
        return this;
    }

    // Set the quaternion to zero
    public final Quaternion zero() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 0.0f;
        return this;
    }

    // Return the orientation matrix corresponding to this quaternion
    public Matrix3x3 getMatrix(Matrix3x3 matrix) {

        float nQ = x * x + y * y + z * z + w * w;
        float s = 0.0f;

        if (nQ > 0.0f) {
            s = 2.0f / nQ;
        }

        // Computations used for optimization (less multiplications)
        float xs = x * s;
        float ys = y * s;
        float zs = z * s;
        float wxs = w * xs;
        float wys = w * ys;
        float wzs = w * zs;
        float xxs = x * xs;
        float xys = x * ys;
        float xzs = x * zs;
        float yys = y * ys;
        float yzs = y * zs;
        float zzs = z * zs;

        // Create the matrix corresponding to the quaternion
        return matrix.set(1.0f - yys - zzs, xys - wzs, xzs + wys,
                xys + wzs, 1.0f - xxs - zzs, yzs - wxs,
                xzs - wys, yzs + wxs, 1.0f - xxs - yys);
    }

    // Compute the rotation angle (in radians) and the rotation axis
    // This method is used to get the rotation angle (in radian) and the unit
    // rotation axis of an orientation quaternion.
    public Vector3 getRotationAngleAxis(Vector3 axis, float[] angle) {

        Quaternion quaternion;

        // If the quaternion is unit
        if (length() == 1.0) {
            quaternion = this;
        } else {
            // We compute the unit quaternion
            quaternion = new Quaternion(this).normalize();
        }

        // Compute the roation angle
        angle[0] = Mathematics.ArcCos(quaternion.w) * 2.0f;

        // Compute the 3D rotation axis
        Vector3 rotationAxis = new Vector3(quaternion.x, quaternion.y, quaternion.z);

        // Normalize the rotation axis
        rotationAxis.normalize();

        // Set the rotation axis values
        return axis.set(rotationAxis);
    }

    // Return the vector v=(x y z) of the quaternion
    public Vector3 getVectorV(Vector3 vector) {
        return vector.set(x, y, z);
    }

    // Overloaded operator for the multiplication with a vector.
    // This methods rotates a point given the rotation of a quaternion.
    public Vector3 multiply(Vector3 vector, Vector3 vectorOut) {
        Quaternion c = new Quaternion(this).conjugate();
        Quaternion p = new Quaternion(vector.x, vector.y, vector.z, 0.0f);
        new Quaternion(this).multiply(p).multiply(c).getVectorV(vectorOut);
        return vectorOut;
    }

    // Compute the spherical linear interpolation between two quaternions.
    // The t argument has to be such that 0 <= t <= 1. This method is static.
    public static void Slerp(Quaternion oldQuaternion, Quaternion newQuaternion2, float t, Quaternion quaternionOut) {

        assert (t >= 0.0f && t <= 1.0f);

        float invert = 1.0f;
        Quaternion tempQ2 = new Quaternion(newQuaternion2);

        // Compute cos(theta) using the quaternion scalar product
        float cosineTheta = oldQuaternion.dot(newQuaternion2);

        // Take care of the sign of cosineTheta
        if (cosineTheta < 0.0f) {
            cosineTheta = -cosineTheta;
            invert = -1.0f;
        }

        // Because of precision, if cos(theta) is nearly 1,
        // therefore theta is nearly 0 and we can write
        // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
        float epsilon = 0.00001f;
        if (1 - cosineTheta < epsilon) {
            quaternionOut.set(oldQuaternion).multiply(1.0f - t).add(tempQ2.multiply(t * invert));
            return;
        }

        // Compute the theta angle
        float theta = Mathematics.ArcCos(cosineTheta);

        // Compute sin(theta)
        float sineTheta = Mathematics.Sin(theta);

        // Compute the two coefficients that are in the spherical linear interpolation formula
        float coeff1 = Mathematics.Sin((1.0f - t) * theta) / sineTheta;
        float coeff2 = Mathematics.Sin(t * theta) / sineTheta * invert;

        // Compute and return the interpolated quaternion
        quaternionOut.set(oldQuaternion).multiply(coeff1).add(tempQ2.multiply(coeff2));
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 59 * hash + Float.floatToIntBits(this.x);
        hash = 59 * hash + Float.floatToIntBits(this.y);
        hash = 59 * hash + Float.floatToIntBits(this.z);
        hash = 59 * hash + Float.floatToIntBits(this.w);
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
        final Quaternion other = (Quaternion) obj;
        if (Float.floatToIntBits(this.x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(this.y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        if (Float.floatToIntBits(this.z) != Float.floatToIntBits(other.z)) {
            return false;
        }
        return Float.floatToIntBits(this.w) == Float.floatToIntBits(other.w);
    }

    @Override
    public String toString() {
        return "(w= " + w + ", x= " + x + ", y= " + y + ", z= " + z + ")";
    }

}
