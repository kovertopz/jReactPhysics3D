package net.smert.jreactphysics3d.common.openglframework.maths;

/**
 * This class represents a 4x4 matrix
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Matrix3 {

    // Elements of the matrix
    private float[][] m = new float[3][3];

    // Constructor
    public Matrix3() {
        setToNull();
    }

    // Constructor
    public Matrix3(float a1, float a2,
            float a3, float b1, float b2, float b3,
            float c1, float c2, float c3) {
        setAllValues(a1, a2, a3, b1, b2, b3, c1, c2, c3);
    }

    // Constructor
    public Matrix3(float[][] n) {
        m[0][0] = n[0][0];
        m[0][1] = n[0][1];
        m[0][2] = n[0][2];
        m[1][0] = n[1][0];
        m[1][1] = n[1][1];
        m[1][2] = n[1][2];
        m[2][0] = n[2][0];
        m[2][1] = n[2][1];
        m[2][2] = n[2][2];
    }

    // Constructor
    public Matrix3(Vector3 a1, Vector3 a2, Vector3 a3) {
        m[0][0] = a1.x;
        m[0][1] = a2.x;
        m[0][2] = a3.x;
        m[1][0] = a1.y;
        m[1][1] = a2.y;
        m[1][2] = a3.y;
        m[2][0] = a1.z;
        m[2][1] = a2.z;
        m[2][2] = a3.z;
    }

    // Constructor
    public Matrix3(Matrix3 matrix) {
        setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2],
                matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
                matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
    }

    // Method to get a value in the matrix
    public float getValue(int i, int j) {
        assert (i >= 0 && i < 3 && j >= 0 && j < 3);
        return m[i][j];
    }

    // Method to set a value in the matrix
    public void setValue(int i, int j, float value) {
        assert (i >= 0 && i < 3 && j >= 0 && j < 3);
        m[i][j] = value;
    }

    // Method to set all the values in the matrix
    public void setAllValues(float a1, float a2, float a3, float b1, float b2, float b3,
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

    // Return a column
    public Vector3 getColumn(int i) {
        assert (i >= 0 && i < 3);
        return new Vector3(m[0][i], m[1][i], m[2][i]);
    }

    // Return the transpose matrix
    public Matrix3 getTranspose() {
        // Return the transpose matrix
        return new Matrix3(m[0][0], m[1][0], m[2][0],
                m[0][1], m[1][1], m[2][1],
                m[0][2], m[1][2], m[2][2]);
    }

    // Return the determinant of the matrix
    public float getDeterminant() {
        // Compute and return the determinant of the matrix
        return (m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) - m[0][1] * (m[1][0] * m[2][2] - m[2][0] * m[1][2])
                + m[0][2] * (m[1][0] * m[2][1] - m[2][0] * m[1][1]));
    }

    // Return the trace of the matrix
    public float getTrace() {
        // Compute and return the trace
        return (m[0][0] + m[1][1] + m[2][2]);
    }

    public void setToNull() {
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

    public boolean isNull() {
        Matrix3 zero = new Matrix3();
        return this == zero;
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

    public boolean isIdentity() {
        Matrix3 I = new Matrix3();
        I.setToIdentity();
        return (this == I);
    }

    // Return the inverse matrix
    public Matrix3 getInverse() {

        // Compute the determinant of the matrix
        float determinant = getDeterminant();

        // Check if the determinant is equal to zero
        assert (determinant > 0.000001f);

        float invDeterminant = 1.0f / determinant;
        Matrix3 tempMatrix = new Matrix3((m[1][1] * m[2][2] - m[2][1] * m[1][2]), -(m[0][1] * m[2][2] - m[2][1] * m[0][2]), (m[0][1] * m[1][2] - m[0][2] * m[1][1]),
                -(m[1][0] * m[2][2] - m[2][0] * m[1][2]), (m[0][0] * m[2][2] - m[2][0] * m[0][2]), -(m[0][0] * m[1][2] - m[1][0] * m[0][2]),
                (m[1][0] * m[2][1] - m[2][0] * m[1][1]), -(m[0][0] * m[2][1] - m[2][0] * m[0][1]), (m[0][0] * m[1][1] - m[0][1] * m[1][0]));

        // Return the inverse matrix
        return tempMatrix.operatorMultiply(invDeterminant);
    }

    // Display the matrix
    public void print() {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                System.out.print(m[i][j] + " ");
            }
            System.out.println("");
        }
    }

    // Overloaded operator =
    public Matrix3 operatorEqual(Matrix3 matrix) {
        if (matrix != this) {
            setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2],
                    matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
                    matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
        }
        return this;
    }

    // Overloaded operator for addition
    public Matrix3 operatorAdd(Matrix3 matrix2) {
        return new Matrix3(m[0][0] + matrix2.m[0][0], m[0][1] + matrix2.m[0][1], m[0][2] + matrix2.m[0][2],
                m[1][0] + matrix2.m[1][0], m[1][1] + matrix2.m[1][1], m[1][2] + matrix2.m[1][2],
                m[2][0] + matrix2.m[2][0], m[2][1] + matrix2.m[2][1], m[2][2] + matrix2.m[2][2]);
    }

    // Overloaded operator for substraction
    public Matrix3 operatorSubtract(Matrix3 matrix2) {
        return new Matrix3(m[0][0] - matrix2.m[0][0], m[0][1] - matrix2.m[0][1], m[0][2] - matrix2.m[0][2],
                m[1][0] - matrix2.m[1][0], m[1][1] - matrix2.m[1][1], m[1][2] - matrix2.m[1][2],
                m[2][0] - matrix2.m[2][0], m[2][1] - matrix2.m[2][1], m[2][2] - matrix2.m[2][2]);
    }

    // Overloaded operator for the negative of the matrix
    public Matrix3 operatorNegative() {
        return new Matrix3(-m[0][0], -m[0][1], -m[0][2],
                -m[1][0], -m[1][1], -m[1][2],
                -m[2][0], -m[2][1], -m[2][2]);
    }

    // Overloaded operator for multiplication with a number
    public Matrix3 operatorMultiply(float nb) {
        return new Matrix3(m[0][0] * nb, m[0][1] * nb, m[0][2] * nb,
                m[1][0] * nb, m[1][1] * nb, m[1][2] * nb,
                m[2][0] * nb, m[2][1] * nb, m[2][2] * nb);
    }

    // Overloaded operator for matrix multiplication
    public Matrix3 operatorMultiply(Matrix3 matrix2) {
        return new Matrix3(m[0][0] * matrix2.m[0][0] + m[0][1] * matrix2.m[1][0] + m[0][2] * matrix2.m[2][0],
                m[0][0] * matrix2.m[0][1] + m[0][1] * matrix2.m[1][1] + m[0][2] * matrix2.m[2][1],
                m[0][0] * matrix2.m[0][2] + m[0][1] * matrix2.m[1][2] + m[0][2] * matrix2.m[2][2],
                m[1][0] * matrix2.m[0][0] + m[1][1] * matrix2.m[1][0] + m[1][2] * matrix2.m[2][0],
                m[1][0] * matrix2.m[0][1] + m[1][1] * matrix2.m[1][1] + m[1][2] * matrix2.m[2][1],
                m[1][0] * matrix2.m[0][2] + m[1][1] * matrix2.m[1][2] + m[1][2] * matrix2.m[2][2],
                m[2][0] * matrix2.m[0][0] + m[2][1] * matrix2.m[1][0] + m[2][2] * matrix2.m[2][0],
                m[2][0] * matrix2.m[0][1] + m[2][1] * matrix2.m[1][1] + m[2][2] * matrix2.m[2][1],
                m[2][0] * matrix2.m[0][2] + m[2][1] * matrix2.m[1][2] + m[2][2] * matrix2.m[2][2]);
    }

    // Overloaded operator for multiplication with a vector
    public Vector3 operatorMultiply(Vector3 vector) {
        return new Vector3(m[0][0] * vector.x + m[0][1] * vector.y + m[0][2] * vector.z,
                m[1][0] * vector.x + m[1][1] * vector.y + m[1][2] * vector.z,
                m[2][0] * vector.x + m[2][1] * vector.y + m[2][2] * vector.z);
    }

    // Overloaded operator for equality condition
    public boolean operatorEquals(Matrix3 matrix) {
        return (m[0][0] == matrix.m[0][0] && m[0][1] == matrix.m[0][1] && m[0][2] == matrix.m[0][2]
                && m[1][0] == matrix.m[1][0] && m[1][1] == matrix.m[1][1] && m[1][2] == matrix.m[1][2]
                && m[2][0] == matrix.m[2][0] && m[2][1] == matrix.m[2][1] && m[2][2] == matrix.m[2][2]);
    }

    // Overloaded operator for the is different condition
    public boolean operatorNotEquals(Matrix3 matrix) {
        return !(this == matrix);
    }

    // Overloaded operator for addition with assignment
    public Matrix3 operatorAddEqual(Matrix3 matrix) {
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
    public Matrix3 operatorSubtractEqual(Matrix3 matrix) {
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
    public Matrix3 operatorMultiplyEqual(float nb) {
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

}
