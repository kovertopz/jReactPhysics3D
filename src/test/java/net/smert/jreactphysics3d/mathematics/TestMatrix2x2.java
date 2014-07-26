package net.smert.jreactphysics3d.mathematics;

import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

/**
 * Unit test for the Matrix2x2 class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestMatrix2x2 {

    // Identity transform
    private Matrix2x2 mIdentity;

    // First example matrix
    private Matrix2x2 mMatrix1;

    @Before
    public void beforeEachTest() {
        mIdentity = Matrix2x2.identity();
        mMatrix1 = new Matrix2x2(2, 24, -4, 5);
    }

    @Test
    public void testClassExists() {
        Assert.assertThat(new Matrix2x2(), CoreMatchers.instanceOf(Matrix2x2.class));
    }

    // Test the constructors
    @Test
    public void testConstructors() {

        Matrix2x2 test1 = new Matrix2x2(5);

        Assert.assertEquals(test1.m[0][0], 5, 0);
        Assert.assertEquals(test1.m[0][1], 5, 0);
        Assert.assertEquals(test1.m[1][0], 5, 0);
        Assert.assertEquals(test1.m[1][1], 5, 0);

        Matrix2x2 test2 = new Matrix2x2(2, 3, 4, 5);

        Assert.assertEquals(test2.m[0][0], 2, 0);
        Assert.assertEquals(test2.m[0][1], 3, 0);
        Assert.assertEquals(test2.m[1][0], 4, 0);
        Assert.assertEquals(test2.m[1][1], 5, 0);

        Matrix2x2 test3 = new Matrix2x2(mMatrix1);

        Assert.assertEquals(test3.equals(mMatrix1), true);

        Matrix2x2 test4 = new Matrix2x2();

        Assert.assertEquals(test4.m[0][0], 0, 0);
        Assert.assertEquals(test4.m[0][1], 0, 0);
        Assert.assertEquals(test4.m[1][0], 0, 0);
        Assert.assertEquals(test4.m[1][1], 0, 0);
    }

    // Test the getter and setter methods
    @Test
    public void testGetSet() {

        // Test method to set all the values
        Matrix2x2 test1 = new Matrix2x2();
        test1.setAllValues(2, 24, -4, 5);

        Assert.assertEquals(test1.equals(mMatrix1), true);

        // Test method to set to zero
        test1.setToZero();

        Assert.assertEquals(test1.equals(new Matrix2x2()), true);

        // Test method that returns a column
        Vector2 column1 = mMatrix1.getColumn(0);
        Vector2 column2 = mMatrix1.getColumn(1);

        Assert.assertEquals(column1.equals(new Vector2(2, -4)), true);
        Assert.assertEquals(column2.equals(new Vector2(24, 5)), true);

        // Test method that returns a row
        Vector2 row1 = mMatrix1.getRow(0);
        Vector2 row2 = mMatrix1.getRow(1);

        Assert.assertEquals(row1.equals(new Vector2(2, 24)), true);
        Assert.assertEquals(row2.equals(new Vector2(-4, 5)), true);
    }

    // Test the identity methods
    @Test
    public void testIdentity() {

        Matrix2x2 test1 = Matrix2x2.identity();

        Assert.assertEquals(test1.m[0][0], 1, 0);
        Assert.assertEquals(test1.m[0][1], 0, 0);
        Assert.assertEquals(test1.m[1][0], 0, 0);
        Assert.assertEquals(test1.m[1][1], 1, 0);

        Matrix2x2 test2 = new Matrix2x2();
        test2.setToIdentity();

        Assert.assertEquals(test2.equals(Matrix2x2.identity()), true);
    }

    @Test
    public void testOthersMethods() {

        // Test transpose
        Matrix2x2 test1 = mMatrix1.getTranspose();

        Assert.assertEquals(test1.equals(new Matrix2x2(2, -4, 24, 5)), true);

        // Test trace
        Assert.assertEquals(mMatrix1.getTrace(), 7, 0);
        Assert.assertEquals(mIdentity.getTrace(), 2, 0);

        // Test determinant
        Matrix2x2 test2 = new Matrix2x2(-24, 64, 253, -35);

        Assert.assertEquals(test2.getDeterminant(), -15352, 0);
        Assert.assertEquals(mMatrix1.getDeterminant(), 106, 0);
        Assert.assertEquals(mIdentity.getDeterminant(), 1, 0);

        // Test inverse
        Matrix2x2 inverseMatrix1 = new Matrix2x2(1, 2, 3, 4).getInverse();

        Assert.assertEquals(inverseMatrix1.m[0][0], -2, 0);
        Assert.assertEquals(inverseMatrix1.m[0][1], 1, 0);
        Assert.assertEquals(inverseMatrix1.m[1][0], 1.5f, 0);
        Assert.assertEquals(inverseMatrix1.m[1][1], -0.5f, 0);

        Matrix2x2 inverseMatrix2 = mMatrix1.getInverse();

        Assert.assertEquals(inverseMatrix2.m[0][0], 0.047169811f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m[0][1], -0.226415094f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m[1][0], 0.037735849f, 10e-6f);
        Assert.assertEquals(inverseMatrix2.m[1][1], 0.018867925f, 10e-6f);

        // Test absolute matrix
        Assert.assertEquals(test2.getAbsoluteMatrix().equals(new Matrix2x2(24, 64, 253, 35)), true);

        Matrix2x2 test3 = new Matrix2x2(-2, -3, -4, -5);

        Assert.assertEquals(test3.getAbsoluteMatrix().equals(new Matrix2x2(2, 3, 4, 5)), true);
    }

    @Test
    public void testOperators() {

        Matrix2x2 matrix1 = new Matrix2x2(2, 3, 4, 5);
        Matrix2x2 matrix2 = new Matrix2x2(-2, 3, -5, 10);

        // Test addition
        Matrix2x2 addition1 = Matrix2x2.operatorAdd(matrix1, matrix2);
        Matrix2x2 addition2 = new Matrix2x2(matrix1);
        addition2.operatorAddEqual(matrix2);

        Assert.assertEquals(addition1.equals(new Matrix2x2(0, 6, -1, 15)), true);
        Assert.assertEquals(addition2.equals(new Matrix2x2(0, 6, -1, 15)), true);

        // Test substraction
        Matrix2x2 substraction1 = Matrix2x2.operatorSubtract(matrix1, matrix2);
        Matrix2x2 substraction2 = new Matrix2x2(matrix1);
        substraction2.operatorSubtractEqual(matrix2);

        Assert.assertEquals(substraction1.equals(new Matrix2x2(4, 0, 9, -5)), true);
        Assert.assertEquals(substraction2.equals(new Matrix2x2(4, 0, 9, -5)), true);

        // Test negative operator
        Matrix2x2 negative = Matrix2x2.operatorNegative(matrix1);

        Assert.assertEquals(negative.equals(new Matrix2x2(-2, -3, -4, -5)), true);

        // Test multiplication with a number
        Matrix2x2 multiplication1 = Matrix2x2.operatorMultiply(3, matrix1);
        Matrix2x2 multiplication2 = Matrix2x2.operatorMultiply(matrix1, 3);
        Matrix2x2 multiplication3 = new Matrix2x2(matrix1);
        multiplication3.operatorMultiplyEqual(3);

        Assert.assertEquals(multiplication1.equals(new Matrix2x2(6, 9, 12, 15)), true);
        Assert.assertEquals(multiplication2.equals(new Matrix2x2(6, 9, 12, 15)), true);
        Assert.assertEquals(multiplication3.equals(new Matrix2x2(6, 9, 12, 15)), true);

        // Test multiplication with a matrix
        Matrix2x2 multiplication4 = Matrix2x2.operatorMultiply(matrix1, matrix2);
        Matrix2x2 multiplication5 = Matrix2x2.operatorMultiply(matrix2, matrix1);

        Assert.assertEquals(multiplication4.equals(new Matrix2x2(-19, 36, -33, 62)), true);
        Assert.assertEquals(multiplication5.equals(new Matrix2x2(8, 9, 30, 35)), true);

        // Test multiplication with a vector
        Vector2 vector1 = new Vector2(3, -32);
        Vector2 vector2 = new Vector2(-31, -422);
        Vector2 test1 = Matrix2x2.operatorMultiply(matrix1, vector1);
        Vector2 test2 = Matrix2x2.operatorMultiply(matrix2, vector2);

        Assert.assertEquals(test1.equals(new Vector2(-90, -148)), true);
        Assert.assertEquals(test2.equals(new Vector2(-1204, -4065)), true);

        // Test equality operators
        Assert.assertEquals(new Matrix2x2(34, 38, 43, 64).operatorEquals(new Matrix2x2(34, 38, 43, 64)), true);
        Assert.assertEquals(new Matrix2x2(34, 64, 43, 7).operatorNotEquals(new Matrix2x2(34, 38, 43, 64)), true);

        // Test operator to read a value
        Assert.assertEquals(mMatrix1.m[0][0], 2, 0);
        Assert.assertEquals(mMatrix1.m[0][1], 24, 0);
        Assert.assertEquals(mMatrix1.m[1][0], -4, 0);
        Assert.assertEquals(mMatrix1.m[1][1], 5, 0);

        // Test operator to set a value
        Matrix2x2 test3 = new Matrix2x2();
        test3.m[0][0] = 2;
        test3.m[0][1] = 24;
        test3.m[1][0] = -4f;
        test3.m[1][1] = 5;

        Assert.assertEquals(test3.equals(mMatrix1), true);
    }

}
