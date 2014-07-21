package net.smert.jreactphysics3d.mathematics;

import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class represents a 2D vector.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Vector2 {

    /// Component x
    public float x;

    /// Component y
    public float y;

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

    // Set the vector to zero
    public void setToZero() {
        x = 0.0f;
        y = 0.0f;
    }

    // Set all the values of the vector
    public void setAllValues(float newX, float newY) {
        x = newX;
        y = newY;
    }

    // Return the length of the vector
    public float length() {
        return (float) Math.sqrt(x * x + y * y);
    }

    // Return the square of the length of the vector
    public float lengthSquare() {
        return x * x + y * y;
    }

    // Scalar product of two vectors (public)
    public float dot(Vector2 vector) {
        return (x * vector.x + y * vector.y);
    }

    // Normalize the vector
    public void normalize() {
        float l = length();
        assert (l > Defaults.MACHINE_EPSILON);
        x /= l;
        y /= l;
    }

    // Return the corresponding absolute value vector
    public Vector2 getAbsoluteVector() {
        return new Vector2(Math.abs(x), Math.abs(y));
    }

    // Return the axis with the minimal value
    public int getMinAxis() {
        return (x < y ? 0 : 1);
    }

    // Return the axis with the maximal value
    public int getMaxAxis() {
        return (x < y ? 1 : 0);
    }

    // Return true if the vector is unit and false otherwise
    public boolean isUnit() {
        return Mathematics.approxEqual(lengthSquare(), 1.0f, Defaults.MACHINE_EPSILON);
    }

    // Return true if the vector is the zero vector
    public boolean isZero() {
        return Mathematics.approxEqual(lengthSquare(), 0.0f, Defaults.MACHINE_EPSILON);
    }

    // Overloaded operator for the equality condition
    public boolean operatorEquals(Vector2 vector) {
        return (x == vector.x && y == vector.y);
    }

    // Overloaded operator for the is different condition
    public boolean operatorNotEquals(Vector2 vector) {
        return !(operatorEquals(vector));
    }

    // Overloaded operator for addition with assignment
    public Vector2 operatorAddEqual(Vector2 vector) {
        x += vector.x;
        y += vector.y;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Vector2 operatorSubtractEqual(Vector2 vector) {
        x -= vector.x;
        y -= vector.y;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public Vector2 operatorMultiplyEqual(float number) {
        x *= number;
        y *= number;
        return this;
    }

    // Overloaded operator for division by a number with assignment
    public Vector2 operatorDivideEqual(float number) {
        assert (number > Defaults.MACHINE_EPSILON);
        x /= number;
        y /= number;
        return this;
    }

    // Overloaded operator for value access
    public float operatorSquareBrackets(int index) {
        if (index == 0) {
            return x;
        } else if (index == 1) {
            return y;
        }
        throw new IllegalArgumentException("Unknown index: " + index);
    }

    // Overloaded operator for addition
    public static Vector2 operatorAdd(Vector2 vector1, Vector2 vector2) {
        return new Vector2(vector1.x + vector2.x, vector1.y + vector2.y);
    }

    // Overloaded operator for substraction
    public static Vector2 operatorSubtract(Vector2 vector1, Vector2 vector2) {
        return new Vector2(vector1.x - vector2.x, vector1.y - vector2.y);
    }

    // Overloaded operator for the negative of a vector
    public static Vector2 operatorNegative(Vector2 vector) {
        return new Vector2(-vector.x, -vector.y);
    }

    // Overloaded operator for multiplication with a number
    public static Vector2 operatorMultiply(Vector2 vector, float number) {
        return new Vector2(number * vector.x, number * vector.y);
    }

    // Overloaded operator for division by a number
    public static Vector2 operatorDivide(Vector2 vector, float number) {
        assert (number > Defaults.MACHINE_EPSILON);
        return new Vector2(vector.x / number, vector.y / number);
    }

    // Overloaded operator for multiplication with a number
    public static Vector2 operatorMultiply(float number, Vector2 vector) {
        return operatorMultiply(vector, number);
    }

    // Assignment operator
    public Vector2 operatorEqual(Vector2 vector) {
        if (vector != this) {
            x = vector.x;
            y = vector.y;
        }
        return this;
    }

}
