package net.smert.jreactphysics3d.common.openglframework.maths;

/**
 * This class represents a 3D vector.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Vector3 {

    // Components of the vector
    public float x, y, z;

    // Constructor
    public Vector3() {
        this(0, 0, 0);
    }

    // Constructor
    public Vector3(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // Constructor
    public Vector3(Vector3 vector) {
        x = vector.x;
        y = vector.y;
        z = vector.z;
    }

    // = operator
    public Vector3 operatorEqual(Vector3 vector) {
        if (vector != this) {
            x = vector.x;
            y = vector.y;
            z = vector.z;
        }
        return this;
    }

    // + operator
    public Vector3 operatorAdd(Vector3 v) {
        return new Vector3(x + v.x, y + v.y, z + v.z);
    }

    // += operator
    public Vector3 operatorAddEqual(Vector3 v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return this;
    }

    // - operator
    public Vector3 operatorSubtract(Vector3 v) {
        return new Vector3(x - v.x, y - v.y, z - v.z);
    }

    // -= operator
    public Vector3 operatorSubtractEqual(Vector3 v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return this;
    }

    // == operator
    public boolean operatorEquals(Vector3 v) {
        return x == v.x && y == v.y && z == v.z;
    }

    // != operator
    public boolean operatorNotEquals(Vector3 v) {
        return !(this == v);
    }

    // * operator
    public Vector3 operatorMultiply(float f) {
        return new Vector3(f * x, f * y, f * z);
    }

    // *= operator
    public Vector3 operatorMultiplyEqual(float f) {
        x *= f;
        y *= f;
        z *= f;
        return this;
    }

    // / operator
    public Vector3 operatorDivide(float f) {
        assert (f > 0.000001f);
        float inv = 1.f / f;
        return new Vector3(x * inv, y * inv, z * inv);
    }

    // /= operator
    public Vector3 operatorDivideEqual(float f) {
        assert (f > 0.000001f);
        float inv = 1.f / f;
        x *= inv;
        y *= inv;
        z *= inv;
        return this;
    }

    // - operator
    public Vector3 operatorNegative() {
        return new Vector3(-x, -y, -z);
    }

    // [] operator
    public float operatorSquareBrackets(int i) {
        assert (i >= 0 && i <= 2);
        switch (i) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
        }
        return z;
    }

    // Cross product operator
    public Vector3 cross(Vector3 v) {
        return new Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    // Dot product operator
    public float dot(Vector3 v) {
        return x * v.x + y * v.y + z * v.z;
    }

    // Normalize the vector and return it
    public Vector3 normalize() {
        float l = length();
        if (l < 0.000001f) {
            assert (false);
        }
        x /= l;
        y /= l;
        z /= l;
        return this;
    }

    public boolean isNull() {
        return (x == 0 && y == 0 && z == 0);
    }

    // Clamp the values between 0 and 1
    public Vector3 clamp01() {
        if (x > 1.f) {
            x = 1.f;
        } else if (x < 0.f) {
            x = 0.f;
        }
        if (y > 1.f) {
            y = 1.f;
        } else if (y < 0.f) {
            y = 0.f;
        }
        if (z > 1.f) {
            z = 1.f;
        } else if (z < 0.f) {
            z = 0.f;
        }
        return this;
    }

    // Return the squared length of the vector
    public float lengthSquared() {
        return x * x + y * y + z * z;
    }

    // Return the length of the vector
    public float length() {
        return (float) Math.sqrt(lengthSquared());
    }

    @Override
    public String toString() {
        return "{" + x + " " + y + " " + z + "}";
    }

}
