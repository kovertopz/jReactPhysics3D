package net.smert.jreactphysics3d.mathematics;

import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

/**
 * Unit test for the Matrix2x2 class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestMatrix2x2 {

    @Test
    public void testClassExists() {
        Assert.assertThat(new Matrix2x2(), CoreMatchers.instanceOf(Matrix2x2.class));
    }

    // Test the constructors
    @Test
    public void testConstructors() {

        Matrix2x2 test1 = new Matrix2x2();

        Assert.assertEquals(test1.m00, 0, 0);
        Assert.assertEquals(test1.m01, 0, 0);
        Assert.assertEquals(test1.m10, 0, 0);
        Assert.assertEquals(test1.m11, 0, 0);

        Matrix2x2 test2 = new Matrix2x2(5);

        Assert.assertEquals(test2.m00, 5, 0);
        Assert.assertEquals(test2.m01, 5, 0);
        Assert.assertEquals(test2.m10, 5, 0);
        Assert.assertEquals(test2.m11, 5, 0);

        Matrix2x2 test3 = new Matrix2x2(2, 3, 4, 5);

        Assert.assertEquals(test3.m00, 2, 0);
        Assert.assertEquals(test3.m01, 3, 0);
        Assert.assertEquals(test3.m10, 4, 0);
        Assert.assertEquals(test3.m11, 5, 0);

        Matrix2x2 matrix = new Matrix2x2(2, 24, -4, 5);
        Matrix2x2 test4 = new Matrix2x2(matrix);

        Assert.assertEquals(test4.m00, 2, 0);
        Assert.assertEquals(test4.m01, 24, 0);
        Assert.assertEquals(test4.m10, -4, 0);
        Assert.assertEquals(test4.m11, 5, 0);
    }

    // Test the getter and setter methods
    @Test
    public void testGetSet() {

        // Test method to set all the values
        Matrix2x2 test1 = new Matrix2x2();
        test1.set(2, 24, -4, 5);

        Assert.assertEquals(test1.m00, 2, 0);
        Assert.assertEquals(test1.m01, 24, 0);
        Assert.assertEquals(test1.m10, -4, 0);
        Assert.assertEquals(test1.m11, 5, 0);

        // Test method to set all values to another matrix
        Matrix2x2 test2 = new Matrix2x2(-10, 55, 102, 13);
        test1.set(test2);

        Assert.assertEquals(test1.m00, -10, 0);
        Assert.assertEquals(test1.m01, 55, 0);
        Assert.assertEquals(test1.m10, 102, 0);
        Assert.assertEquals(test1.m11, 13, 0);

        // Test method that returns a column
        Vector2 column1 = test2.getColumn(0);
        Vector2 column2 = test2.getColumn(1);

        Assert.assertEquals(column1.x, -10, 0);
        Assert.assertEquals(column1.y, 102, 0);
        Assert.assertEquals(column2.x, 55, 0);
        Assert.assertEquals(column2.y, 13, 0);

        // Test method that returns a row
        Vector2 row1 = test2.getRow(0);
        Vector2 row2 = test2.getRow(1);

        Assert.assertEquals(row1.x, -10, 0);
        Assert.assertEquals(row1.y, 55, 0);
        Assert.assertEquals(row2.x, 102, 0);
        Assert.assertEquals(row2.y, 13, 0);
    }

    // Test others methods
    @Test
    public void testOthersMethods() {

        Matrix2x2 test1 = new Matrix2x2(-10, 55, 102, 13);

        // Test method to set to identity
        test1.identity();

        Assert.assertEquals(test1.m00, 1, 0);
        Assert.assertEquals(test1.m01, 0, 0);
        Assert.assertEquals(test1.m10, 0, 0);
        Assert.assertEquals(test1.m11, 1, 0);

        Matrix2x2 test2 = new Matrix2x2(2, 24, -4, 5);

        // Test transpose
        test2.transpose();

        Assert.assertEquals(test2.m00, 2, 0);
        Assert.assertEquals(test2.m01, -4, 0);
        Assert.assertEquals(test2.m10, 24, 0);
        Assert.assertEquals(test2.m11, 5, 0);

        // Test method to set to zero
        test2.zero();

        Assert.assertEquals(test2.m00, 0, 0);
        Assert.assertEquals(test2.m01, 0, 0);
        Assert.assertEquals(test2.m10, 0, 0);
        Assert.assertEquals(test2.m11, 0, 0);

        // Test trace
        Matrix2x2 test3 = new Matrix2x2(2, 24, -4, 5);
        Matrix2x2 test4 = new Matrix2x2();
        test4.identity();

        Assert.assertEquals(test3.getTrace(), 7, 0);
        Assert.assertEquals(test4.getTrace(), 2, 0);

        // Test determinant
        Matrix2x2 test5 = new Matrix2x2(-24, 64, 253, -35);

        Assert.assertEquals(test5.getDeterminant(), -15352, 0);
        Assert.assertEquals(test3.getDeterminant(), 106, 0);
        Assert.assertEquals(test4.getDeterminant(), 1, 0);

        // Test inverse
        Matrix2x2 test6 = new Matrix2x2(1, 2, 3, 4);
        test6.inverse();

        Assert.assertEquals(test6.m00, -2, 0);
        Assert.assertEquals(test6.m01, 1, 0);
        Assert.assertEquals(test6.m10, 1.5f, 0);
        Assert.assertEquals(test6.m11, -0.5f, 0);

        Matrix2x2 test7 = new Matrix2x2(test3);
        test7.inverse();

        Assert.assertEquals(test7.m00, 0.047169811f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m01, -0.226415094f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m10, 0.037735849f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test7.m11, 0.018867925f, TestDefaults.FLOAT_EPSILON);

        // Test absolute matrix
        Matrix2x2 test8 = new Matrix2x2(-24, 64, 253, -35);
        test8.abs();

        Assert.assertEquals(test8.m00, 24, 0);
        Assert.assertEquals(test8.m01, 64, 0);
        Assert.assertEquals(test8.m10, 253, 0);
        Assert.assertEquals(test8.m11, 35, 0);

        Matrix2x2 test9 = new Matrix2x2(-2, -3, -4, -5);
        test9.abs();

        Assert.assertEquals(test9.m00, 2, 0);
        Assert.assertEquals(test9.m01, 3, 0);
        Assert.assertEquals(test9.m10, 4, 0);
        Assert.assertEquals(test9.m11, 5, 0);

        // Test equality
        Matrix2x2 test10 = new Matrix2x2(test5);
        Matrix2x2 test11 = new Matrix2x2(test6);
        Matrix2x2 test12 = new Matrix2x2(test7);
        Matrix2x2 test13 = new Matrix2x2(test8);
        Matrix2x2 test14 = new Matrix2x2(test9);

        Assert.assertEquals(test5.equals(test10), true);
        Assert.assertEquals(test6.equals(test11), true);
        Assert.assertEquals(test7.equals(test12), true);
        Assert.assertEquals(test8.equals(test13), true);
        Assert.assertEquals(test9.equals(test14), true);
        Assert.assertEquals(test9.equals(test10), false);
        Assert.assertEquals(test8.equals(test11), false);
        Assert.assertEquals(test6.equals(test13), false);
        Assert.assertEquals(test5.equals(test14), false);
    }

    // Test the operators
    @Test
    public void testOperators() {

        Matrix2x2 matrix1 = new Matrix2x2(2, 3, 4, 5);
        Matrix2x2 matrix2 = new Matrix2x2(-2, 3, -5, 10);

        // Test addition
        Matrix2x2 addition = new Matrix2x2(matrix1);
        addition.add(matrix2);

        Assert.assertEquals(addition.equals(new Matrix2x2(0, 6, -1, 15)), true);

        // Test substraction
        Matrix2x2 substraction = new Matrix2x2(matrix1);
        substraction.subtract(matrix2);

        Assert.assertEquals(substraction.equals(new Matrix2x2(4, 0, 9, -5)), true);

        // Test negative operator
        Matrix2x2 negative = new Matrix2x2(matrix1);
        negative.invert();

        Assert.assertEquals(negative.equals(new Matrix2x2(-2, -3, -4, -5)), true);

        // Test multiplication with a number
        Matrix2x2 multiplication1 = new Matrix2x2(matrix1);
        multiplication1.multiply(3);

        Assert.assertEquals(multiplication1.equals(new Matrix2x2(6, 9, 12, 15)), true);

        // Test multiplication with a matrix
        Matrix2x2 multiplication2 = new Matrix2x2(matrix1);
        multiplication2.multiply(matrix2);
        Matrix2x2 multiplication3 = new Matrix2x2(matrix2);
        multiplication3.multiply(matrix1);

        Assert.assertEquals(multiplication2.equals(new Matrix2x2(-19, 36, -33, 62)), true);
        Assert.assertEquals(multiplication3.equals(new Matrix2x2(8, 9, 30, 35)), true);

        // Test multiplication with a vector
        Vector2 vector1 = new Vector2(3, -32);
        Vector2 vector2 = new Vector2(-31, -422);
        Vector2 test1 = new Vector2();
        Vector2 test2 = new Vector2();

        matrix1.multiply(vector1, test1);
        matrix2.multiply(vector2, test2);

        Assert.assertEquals(test1.equals(new Vector2(-90, -148)), true);
        Assert.assertEquals(test2.equals(new Vector2(-1204, -4065)), true);
    }

}
