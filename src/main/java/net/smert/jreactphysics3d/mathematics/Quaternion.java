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

    // Create a unit quaternion from a rotation matrix
    public Quaternion(Matrix3x3 matrix) {
        fromMatrix(matrix);
    }

    // Copy-constructor
    public Quaternion(Quaternion quaternion) {
        x = quaternion.x;
        y = quaternion.y;
        z = quaternion.z;
        w = quaternion.w;
    }

    // Constructor with the component w and the vector v=(x y z)
    public Quaternion(Vector3 v, float newW) {
        x = v.x;
        y = v.y;
        z = v.z;
        w = newW;
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

    public Quaternion fromMatrix(Matrix3x3 matrix) {

        // Get the trace of the matrix
        float r, s, trace = matrix.getTrace();

        if (trace < 0.0f) {
            if (matrix.m[1][1] > matrix.m[0][0]) {
                if (matrix.m[2][2] > matrix.m[1][1]) {
                    r = Mathematics.Sqrt(matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1] + 1.0f);
                    s = 0.5f / r;

                    // Compute the quaternion
                    x = (matrix.m[2][0] + matrix.m[0][2]) * s;
                    y = (matrix.m[1][2] + matrix.m[2][1]) * s;
                    z = 0.5f * r;
                    w = (matrix.m[1][0] - matrix.m[0][1]) * s;
                } else {
                    r = Mathematics.Sqrt(matrix.m[1][1] - matrix.m[2][2] - matrix.m[0][0] + 1.0f);
                    s = 0.5f / r;

                    // Compute the quaternion
                    x = (matrix.m[0][1] + matrix.m[1][0]) * s;
                    y = 0.5f * r;
                    z = (matrix.m[1][2] + matrix.m[2][1]) * s;
                    w = (matrix.m[0][2] - matrix.m[2][0]) * s;
                }
            } else if (matrix.m[2][2] > matrix.m[0][0]) {
                r = Mathematics.Sqrt(matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1] + 1.0f);
                s = 0.5f / r;

                // Compute the quaternion
                x = (matrix.m[2][0] + matrix.m[0][2]) * s;
                y = (matrix.m[1][2] + matrix.m[2][1]) * s;
                z = 0.5f * r;
                w = (matrix.m[1][0] - matrix.m[0][1]) * s;
            } else {
                r = Mathematics.Sqrt(matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2] + 1.0f);
                s = 0.5f / r;

                // Compute the quaternion
                x = 0.5f * r;
                y = (matrix.m[0][1] + matrix.m[1][0]) * s;
                z = (matrix.m[2][0] - matrix.m[0][2]) * s;
                w = (matrix.m[2][1] - matrix.m[1][2]) * s;
            }
        } else {
            r = Mathematics.Sqrt(trace + 1.0f);
            s = 0.5f / r;

            // Compute the quaternion
            x = (matrix.m[2][1] - matrix.m[1][2]) * s;
            y = (matrix.m[0][2] - matrix.m[2][0]) * s;
            z = (matrix.m[1][0] - matrix.m[0][1]) * s;
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
        float lengthSquareQuaternion = lengthSquare();
        assert (lengthSquareQuaternion > Defaults.MACHINE_EPSILON);

        // Compute and return the inverse quaternion
        x /= -lengthSquareQuaternion;
        y /= -lengthSquareQuaternion;
        z /= -lengthSquareQuaternion;
        w /= lengthSquareQuaternion;
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
        set(
                newVector.getX(), newVector.getY(), newVector.getZ(),
                w * quaternion.w - q1V.dot(q2V));
        return this;
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
    public Quaternion set(float newX, float newY, float newZ, float newW) {
        x = newX;
        y = newY;
        z = newZ;
        w = newW;
        return this;
    }

    public Quaternion set(Quaternion quaternion) {
        x = quaternion.x;
        y = quaternion.y;
        z = quaternion.z;
        w = quaternion.w;
        return this;
    }

    public Quaternion setW(float newW) {
        w = newW;
        return this;
    }

    public Quaternion setX(float newX) {
        x = newX;
        return this;
    }

    public Quaternion setY(float newY) {
        y = newY;
        return this;
    }

    public Quaternion setZ(float newZ) {
        z = newZ;
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
    public Quaternion zero() {
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
        return matrix.setAllValues(1.0f - yys - zzs, xys - wzs, xzs + wys,
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
    public static void Slerp(Quaternion quaternion1, Quaternion quaternion2, float t, Quaternion quaternionOut) {

        assert (t >= 0.0f && t <= 1.0f);

        float invert = 1.0f;
        Quaternion tempQ2 = new Quaternion(quaternion2);

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
            quaternionOut.set(quaternion1).multiply(1.0f - t).add(tempQ2.multiply(t * invert));
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
        quaternionOut.set(quaternion1).multiply(coeff1).add(tempQ2.multiply(coeff2));
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
