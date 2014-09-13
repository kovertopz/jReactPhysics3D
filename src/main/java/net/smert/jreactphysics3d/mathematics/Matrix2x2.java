package net.smert.jreactphysics3d.mathematics;

import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class represents a 2x2 matrix.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Matrix2x2 {

    // Rows of the matrix (m[row][column])
    float m00;
    float m10;
    float m01;
    float m11;

    // Constructor
    public Matrix2x2() {
        zero();
    }

    // Constructor
    public Matrix2x2(float value) {
        set(value, value, value, value);
    }

    // Constructor with arguments
    public Matrix2x2(float a1, float a2, float b1, float b2) {
        set(a1, a2, b1, b2);
    }

    // Copy-constructor
    public Matrix2x2(Matrix2x2 matrix) {
        set(matrix);
    }

    // Return the determinant of the matrix
    public float getDeterminant() {
        return m00 * m11 - m10 * m01;
    }

    // Return the trace of the matrix
    public float getTrace() {
        return m00 + m11;
    }

    // Return the matrix with absolute values
    public Matrix2x2 abs() {
        m00 = Math.abs(m00);
        m01 = Math.abs(m01);
        m10 = Math.abs(m10);
        m11 = Math.abs(m11);
        return this;
    }

    // Overloaded operator for addition with assignment
    public Matrix2x2 add(Matrix2x2 matrix) {
        m00 += matrix.m00;
        m01 += matrix.m01;
        m10 += matrix.m10;
        m11 += matrix.m11;
        return this;
    }

    // Set the matrix to the identity matrix
    public Matrix2x2 identity() {
        m00 = 1.0f;
        m01 = 0.0f;
        m10 = 0.0f;
        m11 = 1.0f;
        return this;
    }

    // Return the inverse matrix
    public Matrix2x2 inverse() {

        // Compute the determinant of the matrix
        float determinant = getDeterminant();

        // Check if the determinant is equal to zero
        assert (Math.abs(determinant) > Defaults.MACHINE_EPSILON);
        float invDeterminant = 1.0f / determinant;

        set(m11, -m01, -m10, m00);

        // Return the inverse matrix
        return multiply(invDeterminant);
    }

    // Overloaded operator for the negative of the matrix
    public Matrix2x2 invert() {
        m00 = -m00;
        m01 = -m01;
        m10 = -m10;
        m11 = -m11;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public Matrix2x2 multiply(float number) {
        m00 *= number;
        m01 *= number;
        m10 *= number;
        m11 *= number;
        return this;
    }

    // Overloaded operator for matrix multiplication
    public Matrix2x2 multiply(Matrix2x2 matrix) {
        set(
                m00 * matrix.m00 + m01 * matrix.m10,
                m00 * matrix.m01 + m01 * matrix.m11,
                m10 * matrix.m00 + m11 * matrix.m10,
                m10 * matrix.m01 + m11 * matrix.m11);
        return this;
    }

    // Method to set all the values in the matrix
    public Matrix2x2 set(float a1, float a2, float b1, float b2) {
        m00 = a1;
        m01 = a2;
        m10 = b1;
        m11 = b2;
        return this;
    }

    public Matrix2x2 set(Matrix2x2 matrix) {
        m00 = matrix.m00;
        m01 = matrix.m01;
        m10 = matrix.m10;
        m11 = matrix.m11;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Matrix2x2 subtract(Matrix2x2 matrix) {
        m00 -= matrix.m00;
        m01 -= matrix.m01;
        m10 -= matrix.m10;
        m11 -= matrix.m11;
        return this;
    }

    // Return the transpose matrix
    public Matrix2x2 transpose() {
        set(m00, m10, m01, m11);
        return this;
    }

    // Set the matrix to zero
    public Matrix2x2 zero() {
        m00 = 0.0f;
        m01 = 0.0f;
        m10 = 0.0f;
        m11 = 0.0f;
        return this;
    }

    // Return a column
    public Vector2 getColumn(int index) {
        if (index == 0) {
            return new Vector2(m00, m10);
        } else if (index == 1) {
            return new Vector2(m01, m11);
        }
        throw new IllegalArgumentException("Unknown column index: " + index);
    }

    // Return a row
    public Vector2 getRow(int index) {
        if (index == 0) {
            return new Vector2(m00, m01);
        } else if (index == 1) {
            return new Vector2(m10, m11);
        }
        throw new IllegalArgumentException("Unknown row index: " + index);
    }

    // Overloaded operator for multiplication with a vector
    public Vector2 multiply(Vector2 vector, Vector2 vectorOut) {
        return vectorOut.set(
                m00 * vector.x + m01 * vector.y,
                m10 * vector.x + m11 * vector.y);
    }

    @Override
    public int hashCode() {
        int hash = 5;
        hash = 23 * hash + Float.floatToIntBits(this.m00);
        hash = 23 * hash + Float.floatToIntBits(this.m10);
        hash = 23 * hash + Float.floatToIntBits(this.m01);
        hash = 23 * hash + Float.floatToIntBits(this.m11);
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
        final Matrix2x2 other = (Matrix2x2) obj;
        if (Float.floatToIntBits(this.m00) != Float.floatToIntBits(other.m00)) {
            return false;
        }
        if (Float.floatToIntBits(this.m10) != Float.floatToIntBits(other.m10)) {
            return false;
        }
        if (Float.floatToIntBits(this.m01) != Float.floatToIntBits(other.m01)) {
            return false;
        }
        return Float.floatToIntBits(this.m11) == Float.floatToIntBits(other.m11);
    }

    @Override
    public String toString() {
        return "(00= " + m00 + ", 01= " + m01
                + ", 10= " + m10 + ", 11= " + m11 + ")";
    }

}
