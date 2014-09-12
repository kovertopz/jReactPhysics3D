package net.smert.jreactphysics3d.mathematics;

import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class represents a 2D vector.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Vector2 {

    // Component x
    float x;

    // Component y
    float y;

    // Constructor
    public Vector2() {
        x = 0.0f;
        y = 0.0f;
    }

    // Constructor with arguments
    public Vector2(float newX, float newY) {
        x = newX;
        y = newY;
    }

    // Copy-constructor
    public Vector2(Vector2 vector) {
        x = vector.x;
        y = vector.y;
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
    public float dot(Vector2 vector) {
        return x * vector.x + y * vector.y;
    }

    // Overloaded operator for value access
    public float get(int index) {
        if (index == 0) {
            return x;
        } else if (index == 1) {
            return y;
        }
        throw new IllegalArgumentException("Unknown index: " + index);
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    // Return the length of the vector
    public float length() {
        return Mathematics.Sqrt(x * x + y * y);
    }

    // Return the square of the length of the vector
    public float lengthSquare() {
        return x * x + y * y;
    }

    // Return the axis with the maximal value
    public int getMaxAxis() {
        return (x < y ? 1 : 0);
    }

    // Return the axis with the minimal value
    public int getMinAxis() {
        return (x < y ? 0 : 1);
    }

    // Return the corresponding absolute value vector
    public Vector2 abs() {
        x = Math.abs(x);
        y = Math.abs(y);
        return this;
    }

    // Overloaded operator for addition with assignment
    public Vector2 add(Vector2 vector) {
        x += vector.x;
        y += vector.y;
        return this;
    }

    // Overloaded operator for division by a number with assignment
    public Vector2 divide(float number) {
        assert (number > Defaults.MACHINE_EPSILON);
        x /= number;
        y /= number;
        return this;
    }

    // Overloaded operator for the negative of a vector
    public Vector2 invert() {
        x = -x;
        y = -y;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public Vector2 multiply(float number) {
        x *= number;
        y *= number;
        return this;
    }

    // Normalize the vector
    public Vector2 normalize() {
        float len = length();
        assert (len > Defaults.MACHINE_EPSILON);
        x /= len;
        y /= len;
        return this;
    }

    // Set all the values of the vector
    public Vector2 set(float newX, float newY) {
        x = newX;
        y = newY;
        return this;
    }

    // Return one unit orthogonal vector of the current vector
    public Vector2 setUnitOrthogonal() {
        float len = length();
        assert (len > Defaults.MACHINE_EPSILON);
        return new Vector2(-y / len, x / len);
    }

    public Vector2 setX(float newX) {
        x = newX;
        return this;
    }

    public Vector2 setY(float newY) {
        y = newY;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Vector2 subtract(Vector2 vector) {
        x -= vector.x;
        y -= vector.y;
        return this;
    }

    // Set the vector to zero
    public Vector2 zero() {
        x = 0.0f;
        y = 0.0f;
        return this;
    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 53 * hash + Float.floatToIntBits(this.x);
        hash = 53 * hash + Float.floatToIntBits(this.y);
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
        final Vector2 other = (Vector2) obj;
        if (Float.floatToIntBits(this.x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        return Float.floatToIntBits(this.y) == Float.floatToIntBits(other.y);
    }

    @Override
    public String toString() {
        return "(x= " + x + ", y= " + y + ")";
    }

}
