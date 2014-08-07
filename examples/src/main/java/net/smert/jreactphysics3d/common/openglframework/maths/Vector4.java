package net.smert.jreactphysics3d.common.openglframework.maths;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Vector4 {

    // Components of the vector
    public float x, y, z, w;

    // Constructor
    public Vector4() {
        this(0, 0, 0, 0);
    }

    // Constructor
    public Vector4(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    // Constructor
    public Vector4(Vector4 vector) {
        x = vector.x;
        y = vector.x;
        z = vector.x;
        w = vector.x;
    }

    // + operator
    public Vector4 operatorAdd(Vector4 v) {
        return new Vector4(x + v.x, y + v.y, z + v.z, w + v.w);
    }

    // += operator
    public Vector4 operatorAddEqual(Vector4 v) {
        x += v.x;
        y += v.y;
        z += v.z;
        w += v.w;
        return this;
    }

    // - operator
    public Vector4 operatorSubtract(Vector4 v) {
        return new Vector4(x - v.x, y - v.y, z - v.z, w - v.w);
    }

    // -= operator
    public Vector4 operatorSubtractEqual(Vector4 v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        w -= v.w;
        return this;
    }

    // = operator
    public Vector4 operatorEqual(Vector4 vector) {
        if (vector != this) {
            x = vector.x;
            y = vector.y;
            z = vector.z;
            w = vector.w;
        }
        return this;
    }

    // == operator
    public boolean operatorEquals(Vector4 v) {
        return x == v.x && y == v.y && z == v.z && w == v.w;
    }

    // * operator
    public Vector4 operatorMultiply(float f) {
        return new Vector4(f * x, f * y, f * z, f * w);
    }

    // *= operator
    public Vector4 operatorMultiplyEqual(float f) {
        x *= f;
        y *= f;
        z *= f;
        w *= f;
        return this;
    }

    // / operator
    public Vector4 operatorDivide(float f) {
        assert (f != 0);
        float inv = 1.f / f;
        return new Vector4(x * inv, y * inv, z * inv, w * inv);
    }

    // /= operator
    public Vector4 operatorDivideEqual(float f) {
        assert (f != 0);
        float inv = 1.f / f;
        x *= inv;
        y *= inv;
        z *= inv;
        w *= inv;
        return this;
    }

    // - operator
    public Vector4 operatorNegative() {
        return new Vector4(-x, -y, -z, -w);
    }

    // [] operator
    public float operatorSquareBrackets(int i) {
        assert (i >= 0 && i <= 3);
        switch (i) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            case 3:
                return w;
        }
        return w;
    }

    // Dot product operator
    public float dot(Vector4 v) {
        return x * v.x + y * v.y + z * v.z + w * v.w;
    }

    // Multiply two vectors by their components
    public Vector4 componentMul(Vector4 v) {
        return new Vector4(x * v.x, y * v.y, z * v.z, w * v.w);
    }

    // Clamp the values between 0 and 1
    public Vector4 clamp01() {
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
        if (w > 1.f) {
            w = 1.f;
        } else if (w < 0.f) {
            w = 0.f;
        }
        return this;
    }

    // Return the squared length of the vector
    public float lengthSquared() {
        return x * x + y * y + z * z + w * w;
    }

    // Return the length of the vector
    public float length() {
        return (float) Math.sqrt(lengthSquared());
    }

}
