package net.smert.jreactphysics3d.mathematics;

import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class represents a quaternion. We use the notation : q = (x*i, y*j, z*k, w) to represent a quaternion.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Quaternion {

    // Component x
    public float x;

    // Component y
    public float y;

    // Component z
    public float z;

    // Component w
    public float w;

    // Constructor of the class
    public Quaternion() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 0.0f;
    }

    // Constructor with arguments
    public Quaternion(float newX, float newY, float newZ, float newW) {
        x = newX;
        y = newY;
        z = newZ;
        w = newW;
    }

    // Constructor with the component w and the vector v=(x y z)
    public Quaternion(float newW, Vector3 v) {
        x = v.x;
        y = v.y;
        z = v.z;
        w = newW;
    }

    // Copy-constructor
    public Quaternion(Quaternion quaternion) {
        x = quaternion.x;
        y = quaternion.y;
        z = quaternion.z;
        w = quaternion.w;
    }

    // Create a unit quaternion from a rotation matrix
    public Quaternion(Matrix3x3 matrix) {

        // Get the trace of the matrix
        float trace = matrix.getTrace();

        float r;
        float s;

        if (trace < 0.0f) {
            if (matrix.m[1][1] > matrix.m[0][0]) {
                if (matrix.m[2][2] > matrix.m[1][1]) {
                    r = (float) Math.sqrt(matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1] + 1.0f);
                    s = 0.5f / r;

                    // Compute the quaternion
                    x = (matrix.m[2][0] + matrix.m[0][2]) * s;
                    y = (matrix.m[1][2] + matrix.m[2][1]) * s;
                    z = 0.5f * r;
                    w = (matrix.m[1][0] - matrix.m[0][1]) * s;
                } else {
                    r = (float) Math.sqrt(matrix.m[1][1] - matrix.m[2][2] - matrix.m[0][0] + 1.0f);
                    s = 0.5f / r;

                    // Compute the quaternion
                    x = (matrix.m[0][1] + matrix.m[1][0]) * s;
                    y = 0.5f * r;
                    z = (matrix.m[1][2] + matrix.m[2][1]) * s;
                    w = (matrix.m[0][2] - matrix.m[2][0]) * s;
                }
            } else if (matrix.m[2][2] > matrix.m[0][0]) {
                r = (float) Math.sqrt(matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1] + 1.0f);
                s = 0.5f / r;

                // Compute the quaternion
                x = (matrix.m[2][0] + matrix.m[0][2]) * s;
                y = (matrix.m[1][2] + matrix.m[2][1]) * s;
                z = 0.5f * r;
                w = (matrix.m[1][0] - matrix.m[0][1]) * s;
            } else {
                r = (float) Math.sqrt(matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2] + 1.0f);
                s = 0.5f / r;

                // Compute the quaternion
                x = 0.5f * r;
                y = (matrix.m[0][1] + matrix.m[1][0]) * s;
                z = (matrix.m[2][0] - matrix.m[0][2]) * s;
                w = (matrix.m[2][1] - matrix.m[1][2]) * s;
            }
        } else {
            r = (float) Math.sqrt(trace + 1.0f);
            s = 0.5f / r;

            // Compute the quaternion
            x = (matrix.m[2][1] - matrix.m[1][2]) * s;
            y = (matrix.m[0][2] - matrix.m[2][0]) * s;
            z = (matrix.m[1][0] - matrix.m[0][1]) * s;
            w = 0.5f * r;
        }
    }

    // Set all the values
    public void setAllValues(float newX, float newY, float newZ, float newW) {
        x = newX;
        y = newY;
        z = newZ;
        w = newW;
    }

    // Set the quaternion to zero
    public void setToZero() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 0.0f;
    }

    // Set to the identity quaternion
    public void setToIdentity() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 1.0f;
    }

    // Return the vector v=(x y z) of the quaternion
    public Vector3 getVectorV() {

        // Return the vector v
        return new Vector3(x, y, z);
    }

    // Return the length of the quaternion (public )
    public float length() {
        return (float) Math.sqrt(x * x + y * y + z * z + w * w);
    }

    // Return the square of the length of the quaternion
    public float lengthSquare() {
        return x * x + y * y + z * z + w * w;
    }

    // Normalize the quaternion
    public void normalize() {

        float l = length();

        // Check if the length is not equal to zero
        assert (l > Defaults.MACHINE_EPSILON);

        x /= l;
        y /= l;
        z /= l;
        w /= l;
    }

    // Inverse the quaternion
    public void inverse() {

        // Get the square length of the quaternion
        float lengthSquareQuaternion = lengthSquare();

        assert (lengthSquareQuaternion > Defaults.MACHINE_EPSILON);

        // Compute and return the inverse quaternion
        x /= -lengthSquareQuaternion;
        y /= -lengthSquareQuaternion;
        z /= -lengthSquareQuaternion;
        w /= lengthSquareQuaternion;
    }

    // Return the unit quaternion
    public Quaternion getUnit() {
        float lengthQuaternion = length();

        // Check if the length is not equal to zero
        assert (lengthQuaternion > Defaults.MACHINE_EPSILON);

        // Compute and return the unit quaternion
        return new Quaternion(x / lengthQuaternion, y / lengthQuaternion,
                z / lengthQuaternion, w / lengthQuaternion);
    }

    // Return the identity quaternion
    public static Quaternion identity() {
        return new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    }

    // Return the conjugate of the quaternion (public )
    public Quaternion getConjugate() {
        return new Quaternion(-x, -y, -z, w);
    }

    // Return the inverse of the quaternion (public )
    public Quaternion getInverse() {

        float lengthSquareQuaternion = lengthSquare();

        assert (lengthSquareQuaternion > Defaults.MACHINE_EPSILON);

        // Compute and return the inverse quaternion
        return new Quaternion(-x / lengthSquareQuaternion, -y / lengthSquareQuaternion,
                -z / lengthSquareQuaternion, w / lengthSquareQuaternion);
    }

    // Scalar product between two quaternions
    public float dot(Quaternion quaternion) {
        return (x * quaternion.x + y * quaternion.y + z * quaternion.z + w * quaternion.w);
    }

    // Overloaded operator for the addition of two quaternions
    public Quaternion operatorAdd(Quaternion quaternion) {

        // Return the result quaternion
        return new Quaternion(x + quaternion.x, y + quaternion.y, z + quaternion.z, w + quaternion.w);
    }

    // Overloaded operator for the substraction of two quaternions
    public Quaternion operatorSubtract(Quaternion quaternion) {

        // Return the result of the substraction
        return new Quaternion(x - quaternion.x, y - quaternion.y, z - quaternion.z, w - quaternion.w);
    }

    // Overloaded operator for addition with assignment
    public Quaternion operatorAddEqual(Quaternion quaternion) {
        x += quaternion.x;
        y += quaternion.y;
        z += quaternion.z;
        w += quaternion.w;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Quaternion operatorSubtractEqual(Quaternion quaternion) {
        x -= quaternion.x;
        y -= quaternion.y;
        z -= quaternion.z;
        w -= quaternion.w;
        return this;
    }

    // Overloaded operator for the multiplication with a constant
    public Quaternion operatorMultiply(float nb) {
        return new Quaternion(nb * x, nb * y, nb * z, nb * w);
    }

    // Overloaded operator for the multiplication of two quaternions
    public Quaternion operatorMultiply(Quaternion quaternion) {
        return new Quaternion(w * quaternion.w - getVectorV().dot(quaternion.getVectorV()),
                Vector3.operatorMultiply(w, quaternion.getVectorV())
                .operatorAddEqual(Vector3.operatorMultiply(quaternion.w, getVectorV()))
                .operatorAddEqual(getVectorV().cross(quaternion.getVectorV())));
    }

    // Overloaded operator for the multiplication with a vector.
    // This methods rotates a point given the rotation of a quaternion.
    public Vector3 operatorMultiply(Vector3 point) {
        Quaternion p = new Quaternion(point.x, point.y, point.z, 0.0f);
        return operatorMultiply(p).operatorMultiply(getConjugate()).getVectorV();
    }

    // Overloaded operator for the assignment
    public Quaternion operatorEqual(Quaternion quaternion) {

        // Check for self-assignment
        if (this != quaternion) {
            x = quaternion.x;
            y = quaternion.y;
            z = quaternion.z;
            w = quaternion.w;
        }

        // Return this quaternion
        return this;
    }

    // Overloaded operator for equality condition
    public boolean operatorEquals(Quaternion quaternion) {
        return (x == quaternion.x && y == quaternion.y
                && z == quaternion.z && w == quaternion.w);
    }

    // Compute the rotation angle (in radians) and the rotation axis
    // This method is used to get the rotation angle (in radian) and the unit
    // rotation axis of an orientation quaternion.
    public void getRotationAngleAxis(float[] angle, Vector3 axis) {
        Quaternion quaternion;

        // If the quaternion is unit
        if (length() == 1.0) {
            quaternion = this;
        } else {
            // We compute the unit quaternion
            quaternion = getUnit();
        }

        // Compute the roation angle
        angle[0] = (float) Math.acos(quaternion.w) * 2.0f;

        // Compute the 3D rotation axis
        Vector3 rotationAxis = new Vector3(quaternion.x, quaternion.y, quaternion.z);

        // Normalize the rotation axis
        rotationAxis = rotationAxis.getUnit();

        // Set the rotation axis values
        axis.setAllValues(rotationAxis.x, rotationAxis.y, rotationAxis.z);
    }

    // Return the orientation matrix corresponding to this quaternion
    public Matrix3x3 getMatrix() {

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
        return new Matrix3x3(1.0f - yys - zzs, xys - wzs, xzs + wys,
                xys + wzs, 1.0f - xxs - zzs, yzs - wxs,
                xzs - wys, yzs + wxs, 1.0f - xxs - yys);
    }

    // Compute the spherical linear interpolation between two quaternions.
    // The t argument has to be such that 0 <= t <= 1. This method is static.
    public static Quaternion slerp(Quaternion quaternion1, Quaternion quaternion2, float t) {
        assert (t >= 0.0f && t <= 1.0f);

        float invert = 1.0f;

        // Compute cos(theta) using the quaternion scalar product
        float cosineTheta = quaternion1.dot(quaternion2);

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
            return quaternion1.operatorMultiply(1.0f - t).operatorAdd(quaternion2.operatorMultiply(t * invert));
        }

        // Compute the theta angle
        float theta = (float) Math.acos(cosineTheta);

        // Compute sin(theta)
        float sineTheta = (float) Math.sin(theta);

        // Compute the two coefficients that are in the spherical linear interpolation formula
        float coeff1 = (float) Math.sin((1.0f - t) * theta) / sineTheta;
        float coeff2 = (float) Math.sin(t * theta) / sineTheta * invert;

        // Compute and return the interpolated quaternion
        return quaternion1.operatorMultiply(coeff1).operatorAdd(quaternion2.operatorMultiply(coeff2));
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
