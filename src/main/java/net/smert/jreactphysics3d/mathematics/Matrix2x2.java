package net.smert.jreactphysics3d.mathematics;

import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class represents a 2x2 matrix.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Matrix2x2 {

    /// Rows of the matrix;
    public float[][] m = new float[2][2];

    // Constructor of the class Matrix2x2
    public Matrix2x2() {

        // Initialize all values in the matrix to zero
        setAllValues(0.0f, 0.0f, 0.0f, 0.0f);
    }

    // Constructor
    public Matrix2x2(float value) {
        setAllValues(value, value, value, value);
    }

    // Constructor with arguments
    public Matrix2x2(float a1, float a2, float b1, float b2) {

        // Initialize the matrix with the values
        setAllValues(a1, a2, b1, b2);
    }

    // Copy-constructor
    public Matrix2x2(Matrix2x2 matrix) {
        setAllValues(matrix.m[0][0], matrix.m[0][1],
                matrix.m[1][0], matrix.m[1][1]);
    }

    // Method to set all the values in the matrix
    public final void setAllValues(float a1, float a2,
            float b1, float b2) {
        m[0][0] = a1;
        m[0][1] = a2;
        m[1][0] = b1;
        m[1][1] = b2;
    }

    // Set the matrix to zero
    public void setToZero() {
        m[0][0] = 0.0f;
        m[0][1] = 0.0f;
        m[1][0] = 0.0f;
        m[1][1] = 0.0f;
    }

    // Return a column
    public Vector2 getColumn(int i) {
        assert (i >= 0 && i < 2);
        return new Vector2(m[0][i], m[1][i]);
    }

    // Return a row
    public Vector2 getRow(int i) {
        assert (i >= 0 && i < 2);
        return new Vector2(m[i][0], m[i][1]);
    }

    // Return the transpose matrix
    public Matrix2x2 getTranspose() {

        // Return the transpose matrix
        return new Matrix2x2(m[0][0], m[1][0],
                m[0][1], m[1][1]);
    }

    // Return the determinant of the matrix
    public float getDeterminant() {

        // Compute and return the determinant of the matrix
        return m[0][0] * m[1][1] - m[1][0] * m[0][1];
    }

    // Return the trace of the matrix
    public float getTrace() {

        // Compute and return the trace
        return (m[0][0] + m[1][1]);
    }

    // Set the matrix to the identity matrix
    public void setToIdentity() {
        m[0][0] = 1.0f;
        m[0][1] = 0.0f;
        m[1][0] = 0.0f;
        m[1][1] = 1.0f;
    }

    // Return the 2x2 identity matrix
    public static Matrix2x2 identity() {

        // Return the isdentity matrix
        return new Matrix2x2(1.0f, 0.0f, 0.0f, 1.0f);
    }

    // Return the matrix with absolute values
    public Matrix2x2 getAbsoluteMatrix() {
        return new Matrix2x2(Math.abs(m[0][0]), Math.abs(m[0][1]),
                Math.abs(m[1][0]), Math.abs(m[1][1]));
    }

    // Overloaded operator for addition
    public static Matrix2x2 operatorAdd(Matrix2x2 matrix1, Matrix2x2 matrix2) {
        return new Matrix2x2(matrix1.m[0][0] + matrix2.m[0][0],
                matrix1.m[0][1] + matrix2.m[0][1],
                matrix1.m[1][0] + matrix2.m[1][0],
                matrix1.m[1][1] + matrix2.m[1][1]);
    }

    // Overloaded operator for substraction
    public static Matrix2x2 operatorSubtract(Matrix2x2 matrix1, Matrix2x2 matrix2) {
        return new Matrix2x2(matrix1.m[0][0] - matrix2.m[0][0],
                matrix1.m[0][1] - matrix2.m[0][1],
                matrix1.m[1][0] - matrix2.m[1][0],
                matrix1.m[1][1] - matrix2.m[1][1]);
    }

    // Overloaded operator for the negative of the matrix
    public static Matrix2x2 operatorNegative(Matrix2x2 matrix) {
        return new Matrix2x2(-matrix.m[0][0], -matrix.m[0][1],
                -matrix.m[1][0], -matrix.m[1][1]);
    }

    // Overloaded operator for multiplication with a number
    public static Matrix2x2 operatorMultiply(float nb, Matrix2x2 matrix) {
        return new Matrix2x2(matrix.m[0][0] * nb, matrix.m[0][1] * nb,
                matrix.m[1][0] * nb, matrix.m[1][1] * nb);
    }

    // Overloaded operator for multiplication with a matrix
    public static Matrix2x2 operatorMultiply(Matrix2x2 matrix, float nb) {
        return operatorMultiply(nb, matrix);
    }

    // Overloaded operator for matrix multiplication
    public static Matrix2x2 operatorMultiply(Matrix2x2 matrix1, Matrix2x2 matrix2) {
        return new Matrix2x2(matrix1.m[0][0] * matrix2.m[0][0] + matrix1.m[0][1]
                * matrix2.m[1][0],
                matrix1.m[0][0] * matrix2.m[0][1] + matrix1.m[0][1]
                * matrix2.m[1][1],
                matrix1.m[1][0] * matrix2.m[0][0] + matrix1.m[1][1]
                * matrix2.m[1][0],
                matrix1.m[1][0] * matrix2.m[0][1] + matrix1.m[1][1]
                * matrix2.m[1][1]);
    }

    // Overloaded operator for multiplication with a vector
    public Vector2 operatorMultiply(Matrix2x2 matrix, Vector2 vector) {
        return new Vector2(matrix.m[0][0] * vector.x + matrix.m[0][1] * vector.y,
                matrix.m[1][0] * vector.x + matrix.m[1][1] * vector.y);
    }

    // Overloaded operator for equality condition
    public boolean operatorEquals(Matrix2x2 matrix) {
        return (m[0][0] == matrix.m[0][0] && m[0][1] == matrix.m[0][1]
                && m[1][0] == matrix.m[1][0] && m[1][1] == matrix.m[1][1]);
    }

    // Overloaded operator for the is different condition
    public boolean operatorNotEquals(Matrix2x2 matrix) {
        return !(operatorEquals(matrix));
    }

    // Overloaded operator for addition with assignment
    public Matrix2x2 operatorAddEqual(Matrix2x2 matrix) {
        m[0][0] += matrix.m[0][0];
        m[0][1] += matrix.m[0][1];
        m[1][0] += matrix.m[1][0];
        m[1][1] += matrix.m[1][1];
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Matrix2x2 operatorSubtractEqual(Matrix2x2 matrix) {
        m[0][0] -= matrix.m[0][0];
        m[0][1] -= matrix.m[0][1];
        m[1][0] -= matrix.m[1][0];
        m[1][1] -= matrix.m[1][1];
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public Matrix2x2 operatorMultiplyEqual(float nb) {
        m[0][0] *= nb;
        m[0][1] *= nb;
        m[1][0] *= nb;
        m[1][1] *= nb;
        return this;
    }

    // Overloaded operator to return a row of the matrix.
    /// This operator is also used to access a matrix value using the syntax
    /// matrix[row][col].
    public Vector2 operatorSquareBrackets(int row) {
        return new Vector2(m[row][0], m[row][1]);
    }

    // Return the inverse matrix
    public Matrix2x2 getInverse() {

        // Compute the determinant of the matrix
        float determinant = getDeterminant();

        // Check if the determinant is equal to zero
        assert (Math.abs(determinant) > Defaults.MACHINE_EPSILON);

        float invDeterminant = 1.0f / determinant;

        Matrix2x2 tempMatrix = new Matrix2x2(m[1][1], -m[0][1], -m[1][0], m[0][0]);

        // Return the inverse matrix
        return Matrix2x2.operatorMultiply(invDeterminant, tempMatrix);
    }

}
