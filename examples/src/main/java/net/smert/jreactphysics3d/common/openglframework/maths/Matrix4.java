package net.smert.jreactphysics3d.common.openglframework.maths;

/**
 * This class represents a 4x4 matrix
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Matrix4 {

    // Elements of the matrix
    public float[][] m = new float[4][4];

    // Constructor
    public Matrix4() {
        this(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    // Constructor
    public Matrix4(float m_00, float m_01, float m_02, float m_03,
            float m_10, float m_11, float m_12, float m_13,
            float m_20, float m_21, float m_22, float m_23,
            float m_30, float m_31, float m_32, float m_33) {
        m[0][0] = m_00;
        m[0][1] = m_01;
        m[0][2] = m_02;
        m[0][3] = m_03;
        m[1][0] = m_10;
        m[1][1] = m_11;
        m[1][2] = m_12;
        m[1][3] = m_13;
        m[2][0] = m_20;
        m[2][1] = m_21;
        m[2][2] = m_22;
        m[2][3] = m_23;
        m[3][0] = m_30;
        m[3][1] = m_31;
        m[3][2] = m_32;
        m[3][3] = m_33;
    }

    // Constructor
    public Matrix4(float[][] n) {
        m[0][0] = n[0][0];
        m[0][1] = n[0][1];
        m[0][2] = n[0][2];
        m[0][3] = n[0][3];
        m[1][0] = n[1][0];
        m[1][1] = n[1][1];
        m[1][2] = n[1][2];
        m[1][3] = n[1][3];
        m[2][0] = n[2][0];
        m[2][1] = n[2][1];
        m[2][2] = n[2][2];
        m[2][3] = n[2][3];
        m[3][0] = n[3][0];
        m[3][1] = n[3][1];
        m[3][2] = n[3][2];
        m[3][3] = n[3][3];
    }

    // Constructor
    public Matrix4(Vector3 a1, Vector3 a2, Vector3 a3) {
        m[0][0] = a1.x;
        m[0][1] = a2.x;
        m[0][2] = a3.x;
        m[0][3] = 0.f;
        m[1][0] = a1.y;
        m[1][1] = a2.y;
        m[1][2] = a3.y;
        m[1][3] = 0.f;
        m[2][0] = a1.z;
        m[2][1] = a2.z;
        m[2][2] = a3.z;
        m[2][3] = 0.f;
        m[3][0] = 0.f;
        m[3][1] = 0.f;
        m[3][2] = 0.f;
        m[3][3] = 1.f;
    }

    // Constructor
    public Matrix4(Vector4 a1, Vector4 a2, Vector4 a3) {
        m[0][0] = a1.x;
        m[0][1] = a2.x;
        m[0][2] = a3.x;
        m[0][3] = 0.f;
        m[1][0] = a1.y;
        m[1][1] = a2.y;
        m[1][2] = a3.y;
        m[1][3] = 0.f;
        m[2][0] = a1.z;
        m[2][1] = a2.z;
        m[2][2] = a3.z;
        m[2][3] = 0.f;
        m[3][0] = a1.w;
        m[3][1] = a2.w;
        m[3][2] = a3.w;
        m[3][3] = 1.f;
    }

    // Constructor
    public Matrix4(Matrix4 matrix) {

        setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
                matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
                matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
                matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
    }

    // + operator
    public Matrix4 operatorAdd(Matrix4 n) {
        return new Matrix4(m[0][0] + n.m[0][0], m[0][1] + n.m[0][1], m[0][2] + n.m[0][2], m[0][3] + n.m[0][3],
                m[1][0] + n.m[1][0], m[1][1] + n.m[1][1], m[1][2] + n.m[1][2], m[1][3] + n.m[1][3],
                m[2][0] + n.m[2][0], m[2][1] + n.m[2][1], m[2][2] + n.m[2][2], m[2][3] + n.m[2][3],
                m[3][0] + n.m[3][0], m[3][1] + n.m[3][1], m[3][2] + n.m[3][2], m[3][3] + n.m[3][3]);
    }

    // += operator
    public Matrix4 operatorAddEqual(Matrix4 n) {
        m[0][0] += n.m[0][0];
        m[0][1] += n.m[0][1];
        m[0][2] += n.m[0][2];
        m[0][3] += n.m[0][3];
        m[1][0] += n.m[1][0];
        m[1][1] += n.m[1][1];
        m[1][2] += n.m[1][2];
        m[1][3] += n.m[1][3];
        m[2][0] += n.m[2][0];
        m[2][1] += n.m[2][1];
        m[2][2] += n.m[2][2];
        m[2][3] += n.m[2][3];
        m[3][0] += n.m[3][0];
        m[3][1] += n.m[3][1];
        m[3][2] += n.m[3][2];
        m[3][3] += n.m[3][3];
        return this;
    }

    // - operator
    public Matrix4 operatorSubtract(Matrix4 n) {
        return new Matrix4(m[0][0] - n.m[0][0], m[0][1] - n.m[0][1], m[0][2] - n.m[0][2], m[0][3] - n.m[0][3],
                m[1][0] - n.m[1][0], m[1][1] - n.m[1][1], m[1][2] - n.m[1][2], m[1][3] - n.m[1][3],
                m[2][0] - n.m[2][0], m[2][1] - n.m[2][1], m[2][2] - n.m[2][2], m[2][3] - n.m[2][3],
                m[3][0] - n.m[3][0], m[3][1] - n.m[3][1], m[3][2] - n.m[3][2], m[3][3] - n.m[3][3]);
    }

    // -= operator
    public Matrix4 operatorSubtractEqual(Matrix4 n) {
        m[0][0] -= n.m[0][0];
        m[0][1] -= n.m[0][1];
        m[0][2] -= n.m[0][2];
        m[0][3] -= n.m[0][3];
        m[1][0] -= n.m[1][0];
        m[1][1] -= n.m[1][1];
        m[1][2] -= n.m[1][2];
        m[1][3] -= n.m[1][3];
        m[2][0] -= n.m[2][0];
        m[2][1] -= n.m[2][1];
        m[2][2] -= n.m[2][2];
        m[2][3] -= n.m[2][3];
        m[3][0] -= n.m[3][0];
        m[3][1] -= n.m[3][1];
        m[3][2] -= n.m[3][2];
        m[3][3] -= n.m[3][3];
        return this;
    }

    // = operator
    public Matrix4 operatorEqual(Matrix4 matrix) {
        if (matrix != this) {
            setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
                    matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
                    matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
                    matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
        }
        return this;
    }

    // == operator
    public boolean operatorEquals(Matrix4 n) {
        return m[0][0] == n.m[0][0] && m[0][1] == n.m[0][1] && m[0][2] == n.m[0][2] && m[0][3] == n.m[0][3]
                && m[1][0] == n.m[1][0] && m[1][1] == n.m[1][1] && m[1][2] == n.m[1][2] && m[1][3] == n.m[1][3]
                && m[2][0] == n.m[2][0] && m[2][1] == n.m[2][1] && m[2][2] == n.m[2][2] && m[2][3] == n.m[2][3]
                && m[3][0] == n.m[3][0] && m[3][1] == n.m[3][1] && m[3][2] == n.m[3][2] && m[3][3] == n.m[3][3];
    }

    // * operator
    public Matrix4 operatorMultiply(Matrix4 n) {
        Matrix4 o = new Matrix4();
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float v = 0;
                for (int k = 0; k < 4; k++) {
                    v += m[i][k] * n.m[k][j];
                }
                o.m[i][j] = v;
            }
        }
        return o;
    }

    // * operator
    public Vector3 operatorMultiply(Vector3 v) {
        Vector3 u = new Vector3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3],
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3],
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3]);
        float w = m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3];
        return u.operatorDivideEqual(w);
    }

    // * operator
    public Vector4 operatorMultiply(Vector4 v) {
        Vector4 u = new Vector4(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + v.w * m[0][3],
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + v.w * m[1][3],
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + v.w * m[2][3],
                m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + v.w * m[3][3]);
        if (u.w != 0) {
            return u.operatorDivideEqual(u.w);
        } else {
            return u;
        }
    }

    // * operator
    public Matrix4 operatorMultiply(float f) {
        return new Matrix4(m[0][0] * f, m[0][1] * f, m[0][2] * f, m[0][3] * f,
                m[1][0] * f, m[1][1] * f, m[1][2] * f, m[1][3] * f,
                m[2][0] * f, m[2][1] * f, m[2][2] * f, m[2][3] * f,
                m[3][0] * f, m[3][1] * f, m[3][2] * f, m[3][3] * f);
    }

    // * operator
    public Matrix4 operatorMultiplyEqual(float f) {
        m[0][0] *= f;
        m[0][1] *= f;
        m[0][2] *= f;
        m[0][3] *= f;
        m[1][0] *= f;
        m[1][1] *= f;
        m[1][2] *= f;
        m[1][3] *= f;
        m[2][0] *= f;
        m[2][1] *= f;
        m[2][2] *= f;
        m[2][3] *= f;
        m[3][0] *= f;
        m[3][1] *= f;
        m[3][2] *= f;
        m[3][3] *= f;
        return this;
    }

    // / operator
    public Matrix4 operatorDivide(float f) {
        assert (f != 0);
        return new Matrix4(m[0][0] / f, m[0][1] / f, m[0][2] / f, m[0][3] / f,
                m[1][0] / f, m[1][1] / f, m[1][2] / f, m[1][3] / f,
                m[2][0] / f, m[2][1] / f, m[2][2] / f, m[2][3] / f,
                m[3][0] / f, m[3][1] / f, m[3][2] / f, m[3][3] / f);
    }

    // /= operator
    public Matrix4 operatorDivideEqual(float f) {
        assert (f != 0);
        m[0][0] /= f;
        m[0][1] /= f;
        m[0][2] /= f;
        m[0][3] /= f;
        m[1][0] /= f;
        m[1][1] /= f;
        m[1][2] /= f;
        m[1][3] /= f;
        m[2][0] /= f;
        m[2][1] /= f;
        m[2][2] /= f;
        m[2][3] /= f;
        m[3][0] /= f;
        m[3][1] /= f;
        m[3][2] /= f;
        m[3][3] /= f;
        return this;
    }

    // - operator
    public Matrix4 operatorNegative() {
        return new Matrix4(-m[0][0], -m[0][1], -m[0][2], -m[0][3],
                -m[1][0], -m[1][1], -m[1][2], -m[1][3],
                -m[2][0], -m[2][1], -m[2][2], -m[2][3],
                -m[3][0], -m[3][1], -m[3][2], -m[3][3]);
    }

    // Return the transpose matrix
    public Matrix4 getTranspose() {
        return new Matrix4(m[0][0], m[1][0], m[2][0], m[3][0],
                m[0][1], m[1][1], m[2][1], m[3][1],
                m[0][2], m[1][2], m[2][2], m[3][2],
                m[0][3], m[1][3], m[2][3], m[3][3]);
    }

    // Return the 3x3 upper-left matrix
    public Matrix3 getUpperLeft3x3Matrix() {
        return new Matrix3(m[0][0], m[0][1], m[0][2],
                m[1][0], m[1][1], m[1][2],
                m[2][0], m[2][1], m[2][2]);
    }

    // Return the inversed matrix
    public Matrix4 getInverse() {
        int[] indxc = {0, 0, 0, 0};
        int[] indxr = {0, 0, 0, 0};
        int[] ipiv = {0, 0, 0, 0};
        float[][] minv = new float[4][4];
        float temp;

        for (int s = 0; s < 4; s++) {
            for (int t = 0; t < 4; t++) {
                minv[s][t] = m[s][t];
            }
        }

        for (int i = 0; i < 4; i++) {
            int irow = -1, icol = -1;
            float big = 0f;
            // Choose pivot
            for (int j = 0; j < 4; j++) {
                if (ipiv[j] != 1) {
                    for (int k = 0; k < 4; k++) {
                        if (ipiv[k] == 0) {
                            if (Math.abs(minv[j][k]) >= big) {
                                big = Math.abs(minv[j][k]);
                                irow = j;
                                icol = k;
                            }
                        } else if (ipiv[k] > 1) {
                            System.out.println("ERROR: Singular matrix in MatrixInvert");
                        }
                    }
                }
            }
            ++ipiv[icol];
            // Swap rows _irow_ and _icol_ for pivot
            if (irow != icol) {
                for (int k = 0; k < 4; ++k) {
                    temp = minv[irow][k];
                    minv[irow][k] = minv[icol][k];
                    minv[icol][k] = temp;
                }
            }
            indxr[i] = irow;
            indxc[i] = icol;
            if (minv[icol][icol] == 0.) {
                System.out.println("Singular matrix in MatrixInvert");
            }
            // Set $m[icol][icol]$ to one by scaling row _icol_ appropriately
            float pivinv = 1.f / minv[icol][icol];
            minv[icol][icol] = 1.f;
            for (int j = 0; j < 4; j++) {
                minv[icol][j] *= pivinv;
            }

            // Subtract this row from others to zero out their columns
            for (int j = 0; j < 4; j++) {
                if (j != icol) {
                    float save = minv[j][icol];
                    minv[j][icol] = 0;
                    for (int k = 0; k < 4; k++) {
                        minv[j][k] -= minv[icol][k] * save;
                    }
                }
            }
        }
        // Swap columns to reflect permutation
        for (int j = 3; j >= 0; j--) {
            if (indxr[j] != indxc[j]) {
                for (int k = 0; k < 4; k++) {
                    temp = minv[k][indxr[j]];
                    minv[k][indxr[j]] = minv[k][indxc[j]];
                    minv[k][indxc[j]] = temp;
                }
            }
        }
        return new Matrix4(minv);
    }

    // Method to set all the values in the matrix
    public void setAllValues(float a1, float a2, float a3, float a4,
            float b1, float b2, float b3, float b4,
            float c1, float c2, float c3, float c4,
            float d1, float d2, float d3, float d4) {
        m[0][0] = a1;
        m[0][1] = a2;
        m[0][2] = a3;
        m[0][3] = a4;
        m[1][0] = b1;
        m[1][1] = b2;
        m[1][2] = b3;
        m[1][3] = b4;
        m[2][0] = c1;
        m[2][1] = c2;
        m[2][2] = c3;
        m[2][3] = c4;
        m[3][0] = d1;
        m[3][1] = d2;
        m[3][2] = d3;
        m[3][3] = d4;
    }

    // Set the matrix to the identity matrix
    public Matrix4 setToIdentity() {
        m[0][0] = 1.f;
        m[0][1] = 0.f;
        m[0][2] = 0.f;
        m[0][3] = 0.f;
        m[1][0] = 0.f;
        m[1][1] = 1.f;
        m[1][2] = 0.f;
        m[1][3] = 0.f;
        m[2][0] = 0.f;
        m[2][1] = 0.f;
        m[2][2] = 1.f;
        m[2][3] = 0.f;
        m[3][0] = 0.f;
        m[3][1] = 0.f;
        m[3][2] = 0.f;
        m[3][3] = 1.f;
        return this;
    }

    // Display the matrix
    public void print() {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                System.out.print(m[i][j] + " ");
            }
            System.out.println("");
        }
    }

    // Return the pointer to the data array of the matrix
    public float[][] dataBlock() {
        return m;
    }

    // Return a given value from the matrix
    public float getValue(int i, int j) {
        assert (i >= 0 && i < 4 && j >= 0 && j < 4);
        return m[i][j];
    }

    // Return the trace of the matrix
    public float getTrace() {
        // Compute and return the trace
        return (m[0][0] + m[1][1] + m[2][2] + m[3][3]);
    }

    // Return a 4x4 translation matrix
    public static Matrix4 translationMatrix(Vector3 v) {
        return new Matrix4(1, 0, 0, v.x,
                0, 1, 0, v.y,
                0, 0, 1, v.z,
                0, 0, 0, 1);
    }

    // Return a 4x4 rotation matrix
    public static Matrix4 rotationMatrix(Vector3 axis, float angle) {

        float cosA = (float) Math.cos(angle);
        float sinA = (float) Math.sin(angle);
        Matrix4 rotationMatrix = new Matrix4();
        rotationMatrix.setToIdentity();

        rotationMatrix.m[0][0] = cosA + (1 - cosA) * axis.x * axis.x;
        rotationMatrix.m[0][1] = (1 - cosA) * axis.x * axis.y - axis.z * sinA;
        rotationMatrix.m[0][2] = (1 - cosA) * axis.x * axis.z + axis.y * sinA;
        rotationMatrix.m[0][3] = 0.f;

        rotationMatrix.m[1][0] = (1 - cosA) * axis.x * axis.y + axis.z * sinA;
        rotationMatrix.m[1][1] = cosA + (1 - cosA) * axis.y * axis.y;
        rotationMatrix.m[1][2] = (1 - cosA) * axis.y * axis.z - axis.x * sinA;
        rotationMatrix.m[1][3] = 0.f;

        rotationMatrix.m[2][0] = (1 - cosA) * axis.x * axis.z - axis.y * sinA;
        rotationMatrix.m[2][1] = (1 - cosA) * axis.y * axis.z + axis.x * sinA;
        rotationMatrix.m[2][2] = cosA + (1 - cosA) * axis.z * axis.z;
        rotationMatrix.m[2][3] = 0.f;

        rotationMatrix.m[3][0] = 0.f;
        rotationMatrix.m[3][1] = 0.f;
        rotationMatrix.m[3][2] = 0.f;
        rotationMatrix.m[3][3] = 1.f;

        return rotationMatrix;
    }

}
