package net.smert.jreactphysics3d.mathematics;

import java.util.Arrays;
import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class represents a 3x3 matrix.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Matrix3x3 {

    /// Rows of the matrix;
    public float[][] m = new float[3][3];

    // Constructor of the class Matrix3x3
    public Matrix3x3() {
        // Initialize all values in the matrix to zero
        setAllValues(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    // Constructor
    public Matrix3x3(float value) {
        setAllValues(value, value, value, value, value, value, value, value, value);
    }

    // Constructor with arguments
    public Matrix3x3(float a1, float a2, float a3,
            float b1, float b2, float b3,
            float c1, float c2, float c3) {
        // Initialize the matrix with the values
        setAllValues(a1, a2, a3, b1, b2, b3, c1, c2, c3);
    }

    // Copy-constructor
    public Matrix3x3(Matrix3x3 matrix) {
        setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2],
                matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
                matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
    }

    // Method to set all the values in the matrix
    public final void setAllValues(float a1, float a2, float a3,
            float b1, float b2, float b3,
            float c1, float c2, float c3) {
        m[0][0] = a1;
        m[0][1] = a2;
        m[0][2] = a3;
        m[1][0] = b1;
        m[1][1] = b2;
        m[1][2] = b3;
        m[2][0] = c1;
        m[2][1] = c2;
        m[2][2] = c3;
    }

    // Set the matrix to zero
    public void setToZero() {
        m[0][0] = 0.0f;
        m[0][1] = 0.0f;
        m[0][2] = 0.0f;
        m[1][0] = 0.0f;
        m[1][1] = 0.0f;
        m[1][2] = 0.0f;
        m[2][0] = 0.0f;
        m[2][1] = 0.0f;
        m[2][2] = 0.0f;
    }

    // Return a column
    public Vector3 getColumn(int i) {
        assert (i >= 0 && i < 3);
        return new Vector3(m[0][i], m[1][i], m[2][i]);
    }

    // Return a row
    public Vector3 getRow(int i) {
        assert (i >= 0 && i < 3);
        return new Vector3(m[i][0], m[i][1], m[i][2]);
    }

    // Return the transpose matrix
    public Matrix3x3 getTranspose() {

        // Return the transpose matrix
        return new Matrix3x3(m[0][0], m[1][0], m[2][0],
                m[0][1], m[1][1], m[2][1],
                m[0][2], m[1][2], m[2][2]);
    }

    // Return the determinant of the matrix
    public float getDeterminant() {

        // Compute and return the determinant of the matrix
        return (m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2])
                - m[0][1] * (m[1][0] * m[2][2] - m[2][0] * m[1][2])
                + m[0][2] * (m[1][0] * m[2][1] - m[2][0] * m[1][1]));
    }

    // Return the trace of the matrix
    public float getTrace() {

        // Compute and return the trace
        return (m[0][0] + m[1][1] + m[2][2]);
    }

    // Set the matrix to the identity matrix
    public void setToIdentity() {
        m[0][0] = 1.0f;
        m[0][1] = 0.0f;
        m[0][2] = 0.0f;
        m[1][0] = 0.0f;
        m[1][1] = 1.0f;
        m[1][2] = 0.0f;
        m[2][0] = 0.0f;
        m[2][1] = 0.0f;
        m[2][2] = 1.0f;
    }

    // Return the 3x3 identity matrix
    public static Matrix3x3 identity() {

        // Return the isdentity matrix
        return new Matrix3x3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    }

    // Return a skew-symmetric matrix using a given vector that can be used
    // to compute cross product with another vector using matrix multiplication
    public static Matrix3x3 computeSkewSymmetricMatrixForCrossProduct(Vector3 vector) {
        return new Matrix3x3(0.0f, -vector.z, vector.y, vector.z, 0, -vector.x, -vector.y, vector.x, 0.0f);
    }

    // Return the matrix with absolute values
    public Matrix3x3 getAbsoluteMatrix() {
        return new Matrix3x3(Math.abs(m[0][0]), Math.abs(m[0][1]), Math.abs(m[0][2]),
                Math.abs(m[1][0]), Math.abs(m[1][1]), Math.abs(m[1][2]),
                Math.abs(m[2][0]), Math.abs(m[2][1]), Math.abs(m[2][2]));
    }

    // Overloaded operator for addition
    public static Matrix3x3 operatorAdd(Matrix3x3 matrix1, Matrix3x3 matrix2) {
        return new Matrix3x3(matrix1.m[0][0] + matrix2.m[0][0], matrix1.m[0][1]
                + matrix2.m[0][1], matrix1.m[0][2] + matrix2.m[0][2],
                matrix1.m[1][0] + matrix2.m[1][0], matrix1.m[1][1]
                + matrix2.m[1][1], matrix1.m[1][2] + matrix2.m[1][2],
                matrix1.m[2][0] + matrix2.m[2][0], matrix1.m[2][1]
                + matrix2.m[2][1], matrix1.m[2][2] + matrix2.m[2][2]);
    }

    // Overloaded operator for substraction
    public static Matrix3x3 operatorSubtract(Matrix3x3 matrix1, Matrix3x3 matrix2) {
        return new Matrix3x3(matrix1.m[0][0] - matrix2.m[0][0], matrix1.m[0][1]
                - matrix2.m[0][1], matrix1.m[0][2] - matrix2.m[0][2],
                matrix1.m[1][0] - matrix2.m[1][0], matrix1.m[1][1]
                - matrix2.m[1][1], matrix1.m[1][2] - matrix2.m[1][2],
                matrix1.m[2][0] - matrix2.m[2][0], matrix1.m[2][1]
                - matrix2.m[2][1], matrix1.m[2][2] - matrix2.m[2][2]);
    }

    // Overloaded operator for the negative of the matrix
    public static Matrix3x3 operatorNegative(Matrix3x3 matrix) {
        return new Matrix3x3(-matrix.m[0][0], -matrix.m[0][1], -matrix.m[0][2],
                -matrix.m[1][0], -matrix.m[1][1], -matrix.m[1][2],
                -matrix.m[2][0], -matrix.m[2][1], -matrix.m[2][2]);
    }

    // Overloaded operator for multiplication with a number
    public static Matrix3x3 operatorMultiply(float nb, Matrix3x3 matrix) {
        return new Matrix3x3(matrix.m[0][0] * nb, matrix.m[0][1] * nb, matrix.m[0][2] * nb,
                matrix.m[1][0] * nb, matrix.m[1][1] * nb, matrix.m[1][2] * nb,
                matrix.m[2][0] * nb, matrix.m[2][1] * nb, matrix.m[2][2] * nb);
    }

    // Overloaded operator for multiplication with a matrix
    public static Matrix3x3 operatorMultiply(Matrix3x3 matrix, float nb) {
        return operatorMultiply(nb, matrix);
    }

    // Overloaded operator for matrix multiplication
    public static Matrix3x3 operatorMultiply(Matrix3x3 matrix1, Matrix3x3 matrix2) {
        return new Matrix3x3(matrix1.m[0][0] * matrix2.m[0][0] + matrix1.m[0][1]
                * matrix2.m[1][0] + matrix1.m[0][2] * matrix2.m[2][0],
                matrix1.m[0][0] * matrix2.m[0][1] + matrix1.m[0][1]
                * matrix2.m[1][1] + matrix1.m[0][2] * matrix2.m[2][1],
                matrix1.m[0][0] * matrix2.m[0][2] + matrix1.m[0][1]
                * matrix2.m[1][2] + matrix1.m[0][2] * matrix2.m[2][2],
                matrix1.m[1][0] * matrix2.m[0][0] + matrix1.m[1][1]
                * matrix2.m[1][0] + matrix1.m[1][2] * matrix2.m[2][0],
                matrix1.m[1][0] * matrix2.m[0][1] + matrix1.m[1][1]
                * matrix2.m[1][1] + matrix1.m[1][2] * matrix2.m[2][1],
                matrix1.m[1][0] * matrix2.m[0][2] + matrix1.m[1][1]
                * matrix2.m[1][2] + matrix1.m[1][2] * matrix2.m[2][2],
                matrix1.m[2][0] * matrix2.m[0][0] + matrix1.m[2][1]
                * matrix2.m[1][0] + matrix1.m[2][2] * matrix2.m[2][0],
                matrix1.m[2][0] * matrix2.m[0][1] + matrix1.m[2][1]
                * matrix2.m[1][1] + matrix1.m[2][2] * matrix2.m[2][1],
                matrix1.m[2][0] * matrix2.m[0][2] + matrix1.m[2][1]
                * matrix2.m[1][2] + matrix1.m[2][2] * matrix2.m[2][2]);
    }

    // Overloaded operator for multiplication with a vector
    public static Vector3 operatorMultiply(Matrix3x3 matrix, Vector3 vector) {
        return new Vector3(matrix.m[0][0] * vector.x + matrix.m[0][1] * vector.y
                + matrix.m[0][2] * vector.z,
                matrix.m[1][0] * vector.x + matrix.m[1][1] * vector.y
                + matrix.m[1][2] * vector.z,
                matrix.m[2][0] * vector.x + matrix.m[2][1] * vector.y
                + matrix.m[2][2] * vector.z);
    }

    // Overloaded operator for equality condition
    public boolean operatorEquals(Matrix3x3 matrix) {
        return (m[0][0] == matrix.m[0][0] && m[0][1] == matrix.m[0][1]
                && m[0][2] == matrix.m[0][2]
                && m[1][0] == matrix.m[1][0] && m[1][1] == matrix.m[1][1]
                && m[1][2] == matrix.m[1][2]
                && m[2][0] == matrix.m[2][0] && m[2][1] == matrix.m[2][1]
                && m[2][2] == matrix.m[2][2]);
    }

    // Overloaded operator for the is different condition
    public boolean operatorNotEquals(Matrix3x3 matrix) {
        return !(operatorEquals(matrix));
    }

    // Overloaded operator for addition with assignment
    public Matrix3x3 operatorAddEqual(Matrix3x3 matrix) {
        m[0][0] += matrix.m[0][0];
        m[0][1] += matrix.m[0][1];
        m[0][2] += matrix.m[0][2];
        m[1][0] += matrix.m[1][0];
        m[1][1] += matrix.m[1][1];
        m[1][2] += matrix.m[1][2];
        m[2][0] += matrix.m[2][0];
        m[2][1] += matrix.m[2][1];
        m[2][2] += matrix.m[2][2];
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Matrix3x3 operatorSubtractEqual(Matrix3x3 matrix) {
        m[0][0] -= matrix.m[0][0];
        m[0][1] -= matrix.m[0][1];
        m[0][2] -= matrix.m[0][2];
        m[1][0] -= matrix.m[1][0];
        m[1][1] -= matrix.m[1][1];
        m[1][2] -= matrix.m[1][2];
        m[2][0] -= matrix.m[2][0];
        m[2][1] -= matrix.m[2][1];
        m[2][2] -= matrix.m[2][2];
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public Matrix3x3 operatorMultiplyEqual(float nb) {
        m[0][0] *= nb;
        m[0][1] *= nb;
        m[0][2] *= nb;
        m[1][0] *= nb;
        m[1][1] *= nb;
        m[1][2] *= nb;
        m[2][0] *= nb;
        m[2][1] *= nb;
        m[2][2] *= nb;
        return this;
    }

    // Overloaded operator to return a row of the matrix.
    /// This operator is also used to access a matrix value using the syntax
    /// matrix[row][col].
    public Vector3 operatorSquareBrackets(int row) {
        return new Vector3(m[row][0], m[row][1], m[row][2]);
    }

    // Return the inverse matrix
    public Matrix3x3 getInverse() {

        // Compute the determinant of the matrix
        float determinant = getDeterminant();

        // Check if the determinant is equal to zero
        assert (Math.abs(determinant) > Defaults.MACHINE_EPSILON);

        float invDeterminant = 1.0f / determinant;

        Matrix3x3 tempMatrix = new Matrix3x3((m[1][1] * m[2][2] - m[2][1] * m[1][2]),
                -(m[0][1] * m[2][2] - m[2][1] * m[0][2]),
                (m[0][1] * m[1][2] - m[0][2] * m[1][1]),
                -(m[1][0] * m[2][2] - m[2][0] * m[1][2]),
                (m[0][0] * m[2][2] - m[2][0] * m[0][2]),
                -(m[0][0] * m[1][2] - m[1][0] * m[0][2]),
                (m[1][0] * m[2][1] - m[2][0] * m[1][1]),
                -(m[0][0] * m[2][1] - m[2][0] * m[0][1]),
                (m[0][0] * m[1][1] - m[0][1] * m[1][0]));

        // Return the inverse matrix
        return Matrix3x3.operatorMultiply(invDeterminant, tempMatrix);
    }

    @Override
    public int hashCode() {
        int hash = 5;
        hash = 89 * hash + Arrays.deepHashCode(this.m);
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
        final Matrix3x3 other = (Matrix3x3) obj;
        return Arrays.deepEquals(this.m, other.m);
    }

    @Override
    public String toString() {
        return "(00= " + m[0][0] + ", 01= " + m[0][1] + ", 02= " + m[0][2]
                + ", 10= " + m[1][0] + ", 11= " + m[1][1] + ", 12= " + m[1][2]
                + ", 20= " + m[2][0] + ", 21= " + m[2][1] + ", 22= " + m[2][2] + ")";
    }

    public void set(Matrix3x3 matrix) {
        setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2],
                matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
                matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
    }

}
