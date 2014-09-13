package net.smert.jreactphysics3d.mathematics;

import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

/**
 * Unit test for the Matrix3x3 class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestMatrix3x3 {

    @Test
    public void testClassExists() {
        Assert.assertThat(new Matrix3x3(), CoreMatchers.instanceOf(Matrix3x3.class));
    }

    // Test the constructors
    @Test
    public void testConstructors() {

        Matrix3x3 test1 = new Matrix3x3();

        Assert.assertEquals(test1.m00, 0, 0);
        Assert.assertEquals(test1.m01, 0, 0);
        Assert.assertEquals(test1.m02, 0, 0);
        Assert.assertEquals(test1.m10, 0, 0);
        Assert.assertEquals(test1.m11, 0, 0);
        Assert.assertEquals(test1.m12, 0, 0);
        Assert.assertEquals(test1.m20, 0, 0);
        Assert.assertEquals(test1.m21, 0, 0);
        Assert.assertEquals(test1.m22, 0, 0);

        Matrix3x3 test2 = new Matrix3x3(5);

        Assert.assertEquals(test2.m00, 5, 0);
        Assert.assertEquals(test2.m01, 5, 0);
        Assert.assertEquals(test2.m02, 5, 0);
        Assert.assertEquals(test2.m10, 5, 0);
        Assert.assertEquals(test2.m11, 5, 0);
        Assert.assertEquals(test2.m12, 5, 0);
        Assert.assertEquals(test2.m20, 5, 0);
        Assert.assertEquals(test2.m21, 5, 0);
        Assert.assertEquals(test2.m22, 5, 0);

        Matrix3x3 test3 = new Matrix3x3(2, 3, 4, 5, 6, 7, 8, 9, 10);

        Assert.assertEquals(test3.m00, 2, 0);
        Assert.assertEquals(test3.m01, 3, 0);
        Assert.assertEquals(test3.m02, 4, 0);
        Assert.assertEquals(test3.m10, 5, 0);
        Assert.assertEquals(test3.m11, 6, 0);
        Assert.assertEquals(test3.m12, 7, 0);
        Assert.assertEquals(test3.m20, 8, 0);
        Assert.assertEquals(test3.m21, 9, 0);
        Assert.assertEquals(test3.m22, 10, 0);

        Matrix3x3 matrix = new Matrix3x3(2, 24, 4, 5, -6, 234, -15, 11, 66);
        Matrix3x3 test4 = new Matrix3x3(matrix);

        Assert.assertEquals(test4.m00, 2, 0);
        Assert.assertEquals(test4.m01, 24, 0);
        Assert.assertEquals(test4.m02, 4, 0);
        Assert.assertEquals(test4.m10, 5, 0);
        Assert.assertEquals(test4.m11, -6, 0);
        Assert.assertEquals(test4.m12, 234, 0);
        Assert.assertEquals(test4.m20, -15, 0);
        Assert.assertEquals(test4.m21, 11, 0);
        Assert.assertEquals(test4.m22, 66, 0);
    }

    // Test the getter and setter methods
    @Test
    public void testGetSet() {

        // Test method to set all the values
        Matrix3x3 test1 = new Matrix3x3();
        test1.set(2, 24, 4, 5, -6, 234, -15, 11, 66);

        Assert.assertEquals(test1.m00, 2, 0);
        Assert.assertEquals(test1.m01, 24, 0);
        Assert.assertEquals(test1.m02, 4, 0);
        Assert.assertEquals(test1.m10, 5, 0);
        Assert.assertEquals(test1.m11, -6, 0);
        Assert.assertEquals(test1.m12, 234, 0);
        Assert.assertEquals(test1.m20, -15, 0);
        Assert.assertEquals(test1.m21, 11, 0);
        Assert.assertEquals(test1.m22, 66, 0);

        // Test method to set all values to another matrix
        Matrix3x3 test2 = new Matrix3x3(-10, 55, 102, 13, 2, 8, -9, -17, 24);
        test1.set(test2);

        Assert.assertEquals(test1.m00, -10, 0);
        Assert.assertEquals(test1.m01, 55, 0);
        Assert.assertEquals(test1.m02, 102, 0);
        Assert.assertEquals(test1.m10, 13, 0);
        Assert.assertEquals(test1.m11, 2, 0);
        Assert.assertEquals(test1.m12, 8, 0);
        Assert.assertEquals(test1.m20, -9, 0);
        Assert.assertEquals(test1.m21, -17, 0);
        Assert.assertEquals(test1.m22, 24, 0);

        // Test method that returns a column
        Vector3 column1 = test2.getColumn(0);
        Vector3 column2 = test2.getColumn(1);
        Vector3 column3 = test2.getColumn(2);

        Assert.assertEquals(column1.x, -10, 0);
        Assert.assertEquals(column1.y, 13, 0);
        Assert.assertEquals(column1.z, -9, 0);
        Assert.assertEquals(column2.x, 55, 0);
        Assert.assertEquals(column2.y, 2, 0);
        Assert.assertEquals(column2.z, -17, 0);
        Assert.assertEquals(column3.x, 102, 0);
        Assert.assertEquals(column3.y, 8, 0);
        Assert.assertEquals(column3.z, 24, 0);

        // Test method that returns a row
        Vector3 row1 = test2.getRow(0);
        Vector3 row2 = test2.getRow(1);
        Vector3 row3 = test2.getRow(2);

        Assert.assertEquals(row1.x, -10, 0);
        Assert.assertEquals(row1.y, 55, 0);
        Assert.assertEquals(row1.z, 102, 0);
        Assert.assertEquals(row2.x, 13, 0);
        Assert.assertEquals(row2.y, 2, 0);
        Assert.assertEquals(row2.z, 8, 0);
        Assert.assertEquals(row3.x, -9, 0);
        Assert.assertEquals(row3.y, -17, 0);
        Assert.assertEquals(row3.z, 24, 0);
    }

    // Test others methods
    @Test
    public void testOthersMethods() {

        Matrix3x3 test1 = new Matrix3x3(-10, 55, 102, 13, 2, 8, -9, -17, 24);

        // Test method to set to identity
        test1.identity();

        Assert.assertEquals(test1.m00, 1, 0);
        Assert.assertEquals(test1.m01, 0, 0);
        Assert.assertEquals(test1.m02, 0, 0);
        Assert.assertEquals(test1.m10, 0, 0);
        Assert.assertEquals(test1.m11, 1, 0);
        Assert.assertEquals(test1.m12, 0, 0);
        Assert.assertEquals(test1.m20, 0, 0);
        Assert.assertEquals(test1.m21, 0, 0);
        Assert.assertEquals(test1.m22, 1, 0);

        Matrix3x3 test2 = new Matrix3x3(2, 24, 4, 5, -6, 234, -15, 11, 66);

        // Test transpose
        test2.transpose();

        Assert.assertEquals(test2.m00, 2, 0);
        Assert.assertEquals(test2.m01, 5, 0);
        Assert.assertEquals(test2.m02, -15, 0);
        Assert.assertEquals(test2.m10, 24, 0);
        Assert.assertEquals(test2.m11, -6, 0);
        Assert.assertEquals(test2.m12, 11, 0);
        Assert.assertEquals(test2.m20, 4, 0);
        Assert.assertEquals(test2.m21, 234, 0);
        Assert.assertEquals(test2.m22, 66, 0);

        // Test method to set to zero
        test2.zero();

        Assert.assertEquals(test2.m00, 0, 0);
        Assert.assertEquals(test2.m01, 0, 0);
        Assert.assertEquals(test2.m02, 0, 0);
        Assert.assertEquals(test2.m10, 0, 0);
        Assert.assertEquals(test2.m11, 0, 0);
        Assert.assertEquals(test2.m12, 0, 0);
        Assert.assertEquals(test2.m20, 0, 0);
        Assert.assertEquals(test2.m21, 0, 0);
        Assert.assertEquals(test2.m22, 0, 0);

        // Test trace
        Matrix3x3 test3 = new Matrix3x3(2, 24, 4, 5, -6, 234, -15, 11, 66);
        Matrix3x3 test4 = new Matrix3x3();
        test4.identity();

        Assert.assertEquals(test3.getTrace(), 62, 0);
        Assert.assertEquals(test4.getTrace(), 3, 0);

        // Test determinant
        Matrix3x3 test5 = new Matrix3x3(-24, 64, 253, -35, 52, 72, 21, -35, -363);

        Assert.assertEquals(test5.getDeterminant(), -290159, 0);
        Assert.assertEquals(test3.getDeterminant(), -98240, 0);
        Assert.assertEquals(test4.getDeterminant(), 1, 0);

        // Test inverse
        Matrix3x3 test6 = new Matrix3x3(-24, 64, 253, -35, 52, 72, 21, -35, -363);
        test6.inverse();

        Assert.assertEquals(test6.m00, 0.056369f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.m01, -0.049549f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.m02, 0.029460f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.m10, 0.038575f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.m11, -0.011714f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.m12, 0.024562f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.m20, -0.000458f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.m21, -0.001737f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.m22, -0.003419f, TestDefaults.FLOAT_EPSILON);

        Matrix3x3 test7 = new Matrix3x3(test3);
        test7.inverse();

        Assert.assertEquals(test7.m00, 0.030232f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m01, 0.015676f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m02, -0.057410f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m10, 0.039088f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m11, -0.001954f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m12, 0.004560f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m20, 0.000356f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m21, 0.003888f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m22, 0.001344f, TestDefaults.FLOAT_EPSILON);

        // Test absolute matrix
        Matrix3x3 test8 = new Matrix3x3(-24, 64, 253, -35, 52, 72, 21, -35, -363);
        test8.abs();

        Assert.assertEquals(test8.m00, 24, 0);
        Assert.assertEquals(test8.m01, 64, 0);
        Assert.assertEquals(test8.m02, 253, 0);
        Assert.assertEquals(test8.m10, 35, 0);
        Assert.assertEquals(test8.m11, 52, 0);
        Assert.assertEquals(test8.m12, 72, 0);
        Assert.assertEquals(test8.m20, 21, 0);
        Assert.assertEquals(test8.m21, 35, 0);
        Assert.assertEquals(test8.m22, 363, 0);

        Matrix3x3 test9 = new Matrix3x3(-2, -3, -4, -5, -6, -7, -8, -9, -10);
        test9.abs();

        Assert.assertEquals(test9.m00, 2, 0);
        Assert.assertEquals(test9.m01, 3, 0);
        Assert.assertEquals(test9.m02, 4, 0);
        Assert.assertEquals(test9.m10, 5, 0);
        Assert.assertEquals(test9.m11, 6, 0);
        Assert.assertEquals(test9.m12, 7, 0);
        Assert.assertEquals(test9.m20, 8, 0);
        Assert.assertEquals(test9.m21, 9, 0);
        Assert.assertEquals(test9.m22, 10, 0);

        // Test equality
        Matrix3x3 test10 = new Matrix3x3(test5);
        Matrix3x3 test11 = new Matrix3x3(test6);
        Matrix3x3 test12 = new Matrix3x3(test7);
        Matrix3x3 test13 = new Matrix3x3(test8);
        Matrix3x3 test14 = new Matrix3x3(test9);

        Assert.assertEquals(test5.equals(test10), true);
        Assert.assertEquals(test6.equals(test11), true);
        Assert.assertEquals(test7.equals(test12), true);
        Assert.assertEquals(test8.equals(test13), true);
        Assert.assertEquals(test9.equals(test14), true);
        Assert.assertEquals(test9.equals(test10), false);
        Assert.assertEquals(test8.equals(test11), false);
        Assert.assertEquals(test6.equals(test13), false);
        Assert.assertEquals(test5.equals(test14), false);

        // Test method that computes skew-symmetric matrix for cross product
        Vector3 vector1 = new Vector3(3, -5, 6);
        Vector3 vector2 = new Vector3(73, 42, 26);
        Matrix3x3 test15 = new Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(vector1);

        Assert.assertEquals(test15.m00, 0, 0);
        Assert.assertEquals(test15.m01, -6, 0);
        Assert.assertEquals(test15.m02, -5, 0);
        Assert.assertEquals(test15.m10, 6, 0);
        Assert.assertEquals(test15.m11, 0, 0);
        Assert.assertEquals(test15.m12, -3, 0);
        Assert.assertEquals(test15.m20, 5, 0);
        Assert.assertEquals(test15.m21, 3, 0);
        Assert.assertEquals(test15.m22, 0, 0);

        Vector3 crossProduct1 = vector1.cross(vector2);
        Vector3 crossProduct2 = test15.multiply(vector2, new Vector3());

        Assert.assertEquals(crossProduct1.equals(crossProduct2), true);
    }

    // Test the operators
    @Test
    public void testOperators() {

        Matrix3x3 matrix1 = new Matrix3x3(2, 3, 4, 5, 6, 7, 8, 9, 10);
        Matrix3x3 matrix2 = new Matrix3x3(-2, 3, -5, 10, 4, 7, 2, 5, 8);

        // Test addition
        Matrix3x3 addition = new Matrix3x3(matrix1);
        addition.add(matrix2);

        Assert.assertEquals(addition.equals(new Matrix3x3(0, 6, -1, 15, 10, 14, 10, 14, 18)), true);

        // Test substraction
        Matrix3x3 substraction = new Matrix3x3(matrix1);
        substraction.subtract(matrix2);

        Assert.assertEquals(substraction.equals(new Matrix3x3(4, 0, 9, -5, 2, 0, 6, 4, 2)), true);

        // Test negative operator
        Matrix3x3 negative = new Matrix3x3(matrix1);
        negative.invert();

        Assert.assertEquals(negative.equals(new Matrix3x3(-2, -3, -4, -5, -6, -7, -8, -9, -10)), true);

        // Test multiplication with a number
        Matrix3x3 multiplication1 = new Matrix3x3(matrix1);
        multiplication1.multiply(3);

        Assert.assertEquals(multiplication1.equals(new Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30)), true);

        // Test multiplication with a matrix
        Matrix3x3 multiplication2 = new Matrix3x3(matrix1);
        multiplication2.multiply(matrix2);
        Matrix3x3 multiplication3 = new Matrix3x3(matrix2);
        multiplication3.multiply(matrix1);

        Assert.assertEquals(multiplication2.equals(new Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103)), true);
        Assert.assertEquals(multiplication3.equals(new Matrix3x3(-29, -33, -37, 96, 117, 138, 93, 108, 123)), true);

        // Test multiplication with a vector
        Vector3 vector1 = new Vector3(3, -32, 59);
        Vector3 vector2 = new Vector3(-31, -422, 34);
        Vector3 test1 = new Vector3();
        Vector3 test2 = new Vector3();

        matrix1.multiply(vector1, test1);
        matrix2.multiply(vector2, test2);

        Assert.assertEquals(test1.equals(new Vector3(146, 236, 326)), true);
        Assert.assertEquals(test2.equals(new Vector3(-1374, -1760, -1900)), true);
    }

}
