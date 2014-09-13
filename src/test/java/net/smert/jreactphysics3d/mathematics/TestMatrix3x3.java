package net.smert.jreactphysics3d.mathematics;

import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

/**
 * Unit test for the Matrix3x3 class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestMatrix3x3 {

    // Identity transform
    private Matrix3x3 mIdentity;

    // First example matrix
    private Matrix3x3 mMatrix1;
    
    @Before
    public void beforeEachTest() {
        mIdentity = new Matrix3x3().identity();
        mMatrix1 = new Matrix3x3(2, 24, 4, 5, -6, 234, -15, 11, 66);
    }
    
    @Test
    public void testClassExists() {
        Assert.assertThat(new Matrix3x3(), CoreMatchers.instanceOf(Matrix3x3.class));
    }

    // Test the constructors
    @Test
    public void testConstructors() {
        
        Matrix3x3 test1 = new Matrix3x3(5);
        
        Assert.assertEquals(test1.m00, 5, 0);
        Assert.assertEquals(test1.m01, 5, 0);
        Assert.assertEquals(test1.m02, 5, 0);
        Assert.assertEquals(test1.m10, 5, 0);
        Assert.assertEquals(test1.m11, 5, 0);
        Assert.assertEquals(test1.m12, 5, 0);
        Assert.assertEquals(test1.m20, 5, 0);
        Assert.assertEquals(test1.m21, 5, 0);
        Assert.assertEquals(test1.m22, 5, 0);
        
        Matrix3x3 test2 = new Matrix3x3(2, 3, 4, 5, 6, 7, 8, 9, 10);
        
        Assert.assertEquals(test2.m00, 2, 0);
        Assert.assertEquals(test2.m01, 3, 0);
        Assert.assertEquals(test2.m02, 4, 0);
        Assert.assertEquals(test2.m10, 5, 0);
        Assert.assertEquals(test2.m11, 6, 0);
        Assert.assertEquals(test2.m12, 7, 0);
        Assert.assertEquals(test2.m20, 8, 0);
        Assert.assertEquals(test2.m21, 9, 0);
        Assert.assertEquals(test2.m22, 10, 0);
        
        Matrix3x3 test3 = new Matrix3x3(mMatrix1);
        
        Assert.assertEquals(test3.equals(mMatrix1), true);
        
        Matrix3x3 test4 = new Matrix3x3();
        
        Assert.assertEquals(test4.m00, 0, 0);
        Assert.assertEquals(test4.m01, 0, 0);
        Assert.assertEquals(test4.m02, 0, 0);
        Assert.assertEquals(test4.m10, 0, 0);
        Assert.assertEquals(test4.m11, 0, 0);
        Assert.assertEquals(test4.m12, 0, 0);
        Assert.assertEquals(test4.m20, 0, 0);
        Assert.assertEquals(test4.m21, 0, 0);
        Assert.assertEquals(test4.m22, 0, 0);
    }

    // Test the getter and setter methods
    @Test
    public void testGetSet() {

        // Test method to set all the values
        Matrix3x3 test1 = new Matrix3x3();
        test1.set(2, 24, 4, 5, -6, 234, -15, 11, 66);
        
        Assert.assertEquals(test1.equals(mMatrix1), true);

        // Test method to set to zero
        test1.zero();
        
        Assert.assertEquals(test1.equals(new Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0)), true);

        // Test method that returns a column
        Vector3 column1 = mMatrix1.getColumn(0);
        Vector3 column2 = mMatrix1.getColumn(1);
        Vector3 column3 = mMatrix1.getColumn(2);
        
        Assert.assertEquals(column1.equals(new Vector3(2, 5, -15)), true);
        Assert.assertEquals(column2.equals(new Vector3(24, -6, 11)), true);
        Assert.assertEquals(column3.equals(new Vector3(4, 234, 66)), true);

        // Test method that returns a row
        Vector3 row1 = mMatrix1.getRow(0);
        Vector3 row2 = mMatrix1.getRow(1);
        Vector3 row3 = mMatrix1.getRow(2);
        
        Assert.assertEquals(row1.equals(new Vector3(2, 24, 4)), true);
        Assert.assertEquals(row2.equals(new Vector3(5, -6, 234)), true);
        Assert.assertEquals(row3.equals(new Vector3(-15, 11, 66)), true);
    }

    // Test the identity methods
    public void testIdentity() {
        
        Matrix3x3 test1 = new Matrix3x3().identity();
        
        Assert.assertEquals(test1.m00, 1, 0);
        Assert.assertEquals(test1.m01, 0, 0);
        Assert.assertEquals(test1.m02, 0, 0);
        Assert.assertEquals(test1.m10, 0, 0);
        Assert.assertEquals(test1.m11, 1, 0);
        Assert.assertEquals(test1.m12, 0, 0);
        Assert.assertEquals(test1.m20, 0, 0);
        Assert.assertEquals(test1.m21, 0, 0);
        Assert.assertEquals(test1.m22, 1, 0);
        
        Matrix3x3 test2 = new Matrix3x3();
        test2.identity();
        
        Assert.assertEquals(test2.equals(new Matrix3x3().identity()), true);
    }

    // Test others methods
    @Test
    public void testOthersMethods() {

        // Test transpose
        Matrix3x3 test1 = mMatrix1.getTranspose();
        
        Assert.assertEquals(test1.equals(new Matrix3x3(2, 5, -15, 24, -6, 11, 4, 234, 66)), true);

        // Test trace
        Assert.assertEquals(mMatrix1.getTrace(), 62, 0);
        Assert.assertEquals(mIdentity.getTrace(), 3, 0);

        // Test determinant
        Matrix3x3 test2 = new Matrix3x3(-24, 64, 253, -35, 52, 72, 21, -35, -363);
        
        Assert.assertEquals(test2.getDeterminant(), -290159, 0);
        Assert.assertEquals(mMatrix1.getDeterminant(), -98240, 0);
        Assert.assertEquals(mIdentity.getDeterminant(), 1, 0);

        // Test inverse
        Matrix3x3 inverseMatrix1 = new Matrix3x3(test2).inverse();
        
        Assert.assertEquals(inverseMatrix1.m00, 0.056369f, 10e-6f);
        Assert.assertEquals(inverseMatrix1.m01, -0.049549f, 10e-6f);
        Assert.assertEquals(inverseMatrix1.m02, 0.029460f, 10e-6f);
        Assert.assertEquals(inverseMatrix1.m10, 0.038575f, 10e-6f);
        Assert.assertEquals(inverseMatrix1.m11, -0.011714f, 10e-6f);
        Assert.assertEquals(inverseMatrix1.m12, 0.024562f, 10e-6f);
        Assert.assertEquals(inverseMatrix1.m20, -0.000458f, 10e-6f);
        Assert.assertEquals(inverseMatrix1.m21, -0.001737f, 10e-6f);
        Assert.assertEquals(inverseMatrix1.m22, -0.003419f, 10e-6f);
        
        Matrix3x3 inverseMatrix2 = new Matrix3x3(mMatrix1).inverse();
        
        Assert.assertEquals(inverseMatrix2.m00, 0.030232f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m01, 0.015676f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m02, -0.057410f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m10, 0.039088f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m11, -0.001954f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m12, 0.004560f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m20, 0.000356f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m21, 0.003888f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m22, 0.001344f, 10e-6f);

        // Test absolute matrix
        Assert.assertEquals(new Matrix3x3(test2).abs().equals(new Matrix3x3(24, 64, 253, 35, 52, 72, 21, 35, 363)), true);
        
        Matrix3x3 test3 = new Matrix3x3(-2, -3, -4, -5, -6, -7, -8, -9, -10);
        
        Assert.assertEquals(new Matrix3x3(test3).abs().equals(new Matrix3x3(2, 3, 4, 5, 6, 7, 8, 9, 10)), true);

        // Test method that computes skew-symmetric matrix for cross product
        Vector3 vector1 = new Vector3(3, -5, 6);
        Vector3 vector2 = new Vector3(73, 42, 26);
        Matrix3x3 skewMatrix = new Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(vector1);
        
        Assert.assertEquals(skewMatrix.equals(new Matrix3x3(0, -6, -5, 6, 0, -3, 5, 3, 0)), true);
        
        Vector3 crossProduct1 = vector1.cross(vector2);
        Vector3 crossProduct2 = Matrix3x3.operatorMultiply(skewMatrix, vector2);
        
        Assert.assertEquals(crossProduct1.equals(crossProduct2), true);
    }

    // Test the operators
    @Test
    public void testOperators() {

        // Test addition
        Matrix3x3 matrix1 = new Matrix3x3(2, 3, 4, 5, 6, 7, 8, 9, 10);
        Matrix3x3 matrix2 = new Matrix3x3(-2, 3, -5, 10, 4, 7, 2, 5, 8);
        Matrix3x3 addition1 = Matrix3x3.operatorAdd(matrix1, matrix2);
        Matrix3x3 addition2 = new Matrix3x3(matrix1);
        addition2.add(matrix2);
        
        Assert.assertEquals(addition1.equals(new Matrix3x3(0, 6, -1, 15, 10, 14, 10, 14, 18)), true);
        Assert.assertEquals(addition2.equals(new Matrix3x3(0, 6, -1, 15, 10, 14, 10, 14, 18)), true);

        // Test substraction
        Matrix3x3 substraction1 = Matrix3x3.operatorSubtract(matrix1, matrix2);
        Matrix3x3 substraction2 = new Matrix3x3(matrix1);
        substraction2.subtract(matrix2);
        
        Assert.assertEquals(substraction1.equals(new Matrix3x3(4, 0, 9, -5, 2, 0, 6, 4, 2)), true);
        Assert.assertEquals(substraction2.equals(new Matrix3x3(4, 0, 9, -5, 2, 0, 6, 4, 2)), true);

        // Test negative operator
        Matrix3x3 negative = new Matrix3x3(matrix1).invert();
        
        Assert.assertEquals(negative.equals(new Matrix3x3(-2, -3, -4, -5, -6, -7, -8, -9, -10)), true);

        // Test multiplication with a number
        Matrix3x3 multiplication1 = Matrix3x3.operatorMultiply(3, matrix1);
        Matrix3x3 multiplication2 = Matrix3x3.operatorMultiply(matrix1, 3);
        Matrix3x3 multiplication3 = new Matrix3x3(matrix1);
        multiplication3.multiply(3);
        
        Assert.assertEquals(multiplication1.equals(new Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30)), true);
        Assert.assertEquals(multiplication2.equals(new Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30)), true);
        Assert.assertEquals(multiplication3.equals(new Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30)), true);

        // Test multiplication with a matrix
        Matrix3x3 multiplication4 = Matrix3x3.operatorMultiply(matrix1, matrix2);
        Matrix3x3 multiplication5 = Matrix3x3.operatorMultiply(matrix2, matrix1);
        
        Assert.assertEquals(multiplication4.equals(new Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103)), true);
        Assert.assertEquals(multiplication5.equals(new Matrix3x3(-29, -33, -37, 96, 117, 138, 93, 108, 123)), true);

        // Test multiplication with a vector
        Vector3 vector1 = new Vector3(3, -32, 59);
        Vector3 vector2 = new Vector3(-31, -422, 34);
        Vector3 test1 = Matrix3x3.operatorMultiply(matrix1, vector1);
        Vector3 test2 = Matrix3x3.operatorMultiply(matrix2, vector2);
        
        Assert.assertEquals(test1.equals(new Vector3(146, 236, 326)), true);
        Assert.assertEquals(test2.equals(new Vector3(-1374, -1760, -1900)), true);

        // Test equality operators
        Assert.assertEquals(new Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103).equals(
                new Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103)), true);
        Assert.assertEquals(new Matrix3x3(34, 64, 43, 7, -1, 73, 94, 110, 103).equals(
                new Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103)), false);

        // Test operator to read a value
        Assert.assertEquals(mMatrix1.m00, 2, 0);
        Assert.assertEquals(mMatrix1.m01, 24, 0);
        Assert.assertEquals(mMatrix1.m02, 4, 0);
        Assert.assertEquals(mMatrix1.m10, 5, 0);
        Assert.assertEquals(mMatrix1.m11, -6, 0);
        Assert.assertEquals(mMatrix1.m12, 234, 0);
        Assert.assertEquals(mMatrix1.m20, -15, 0);
        Assert.assertEquals(mMatrix1.m21, 11, 0);
        Assert.assertEquals(mMatrix1.m22, 66, 0);

        // Test operator to set a value
        Matrix3x3 test3 = new Matrix3x3();
        test3.m00 = 2;
        test3.m01 = 24;
        test3.m02 = 4;
        test3.m10 = 5;
        test3.m11 = -6;
        test3.m12 = 234;
        test3.m20 = -15;
        test3.m21 = 11;
        test3.m22 = 66;
        
        Assert.assertEquals(test3.equals(mMatrix1), true);
    }
    
}
