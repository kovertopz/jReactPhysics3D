package net.smert.jreactphysics3d.common.openglframework.maths;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Vector2 {

    // Components of the vector
    public float x, y;

    // Constructor
    public Vector2() {
        this(0, 0);
    }

    // Constructor
    public Vector2(float x, float y) {
        this.x = x;
        this.y = y;
    }

    // Constructor
    public Vector2(Vector2 vector) {
        x = vector.x;
        y = vector.y;
    }

    // + operator
    public Vector2 operatorAdd(Vector2 v) {
        return new Vector2(x + v.x, y + v.y);
    }

    // += operator
    public Vector2 operatorAddEqual(Vector2 v) {
        x += v.x;
        y += v.y;
        return this;
    }

    // - operator
    public Vector2 operatorSubtract(Vector2 v) {
        return new Vector2(x - v.x, y - v.y);
    }

    // -= operator
    public Vector2 operatorSubtractEqual(Vector2 v) {
        x -= v.x;
        y -= v.y;
        return this;
    }

    // = operator
    public Vector2 operatorEqual(Vector2 vector) {
        if (vector != this) {
            x = vector.x;
            y = vector.y;
        }
        return this;
    }

    // == operator
    public boolean operatorEquals(Vector2 v) {
        return x == v.x & y == v.y;
    }

    // * operator
    public Vector2 operatorMultiply(float f) {
        return new Vector2(f * x, f * y);
    }

    // *= operator
    public Vector2 operatorMultiplyEqual(float f) {
        x *= f;
        y *= f;
        return this;
    }

    // / operator
    public Vector2 operatorDivide(float f) {
        assert (f != 0);
        float inv = 1.f / f;
        return new Vector2(x * inv, y * inv);
    }

    // /= operator
    public Vector2 operatorDivideEqual(float f) {
        assert (f != 0);
        float inv = 1.f / f;
        x *= inv;
        y *= inv;
        return this;
    }

    // - operator
    public Vector2 operatorNegative() {
        return new Vector2(-x, -y);
    }

    // [] operator
    public float operatorSquareBrackets(int i) {
        assert (i >= 0 && i <= 1);
        switch (i) {
            case 0:
                return x;
            case 1:
                return y;
        }
        return y;
    }

    // Normalize the vector and return it
    public Vector2 normalize() {
        float l = length();
        assert (l > 0);
        x /= l;
        y /= l;
        return this;
    }

    // Clamp the vector values between 0 and 1
    public Vector2 clamp01() {
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
        return this;
    }

    // Return the squared length of the vector
    public float lengthSquared() {
        return x * x + y * y;
    }

    // Return the length of the vector
    public float length() {
        return (float) Math.sqrt(lengthSquared());
    }

}
