package net.smert.jreactphysics3d.mathematics;

import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

/**
 * Unit test for the Vector2 class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestVector2 {

    // Zero vector
    private Vector2 mVectorZero;

    // Vector (3, 4)
    private Vector2 mVector34;

    @Before
    public void beforeEachTest() {
        mVectorZero = new Vector2(0, 0);
        mVector34 = new Vector2(3, 4);
    }

    @Test
    public void testClassExists() {
        Assert.assertThat(new Vector2(), CoreMatchers.instanceOf(Vector2.class));
    }

    @Test
    // Test the constructors, getter and setter
    public void testConstructors() {

        // Test constructor
        Assert.assertEquals(mVectorZero.x, 0, 0);
        Assert.assertEquals(mVectorZero.y, 0, 0);
        Assert.assertEquals(mVector34.x, 3, 0);
        Assert.assertEquals(mVector34.y, 4, 0);

        // Test copy-constructor
        Vector2 newVector = new Vector2(mVector34);

        Assert.assertEquals(newVector.x, 3, 0);
        Assert.assertEquals(newVector.y, 4, 0);

        // Test method to set values
        Vector2 newVector2 = new Vector2();
        newVector2.setAllValues(6.1f, 7.2f);

        Assert.assertEquals(newVector2.x, 6.1f, 0);
        Assert.assertEquals(newVector2.y, 7.2f, 0);

        // Test method to set to zero
        newVector2.setToZero();

        Assert.assertEquals(newVector2.equals(new Vector2(0, 0)), true);
    }

    @Test
    // Test the length, unit vector and normalize methods
    public void testLengthMethods() {

        // Test length methods
        Assert.assertEquals(mVectorZero.length(), 0, 0);
        Assert.assertEquals(mVectorZero.lengthSquare(), 0, 0);
        Assert.assertEquals(new Vector2(1, 0).length(), 1, 0);
        Assert.assertEquals(new Vector2(0, 1).length(), 1, 0);
        Assert.assertEquals(mVector34.lengthSquare(), 25, 0);

        // Test unit vector methods
        Assert.assertEquals(new Vector2(1, 0).isUnit(), true);
        Assert.assertEquals(new Vector2(0, 1).isUnit(), true);
        Assert.assertEquals(!mVector34.isUnit(), true);
        Assert.assertEquals(new Vector2(5, 0).getUnit().equals(new Vector2(1, 0)), true);
        Assert.assertEquals(new Vector2(0, 5).getUnit().equals(new Vector2(0, 1)), true);

        Assert.assertEquals(!mVector34.isZero(), true);
        Assert.assertEquals(mVectorZero.isZero(), true);

        // Test normalization method
        Vector2 mVector10 = new Vector2(1, 0);
        Vector2 mVector01 = new Vector2(0, 1);
        Vector2 mVector50 = new Vector2(5, 0);
        Vector2 mVector05 = new Vector2(0, 5);
        mVector10.normalize();
        mVector01.normalize();
        mVector50.normalize();
        mVector05.normalize();

        Assert.assertEquals(mVector10.equals(new Vector2(1, 0)), true);
        Assert.assertEquals(mVector01.equals(new Vector2(0, 1)), true);
        Assert.assertEquals(mVector50.equals(new Vector2(1, 0)), true);
        Assert.assertEquals(mVector05.equals(new Vector2(0, 1)), true);
    }

    @Test
    // Test the dot product
    public void testDotProduct() {

        // Test the dot product
        Assert.assertEquals(new Vector2(5, 0).dot(new Vector2(0, 8)), 0, 0);
        Assert.assertEquals(new Vector2(5, 8).dot(new Vector2(0, 0)), 0, 0);
        Assert.assertEquals(new Vector2(12, 45).dot(new Vector2(0, 0)), 0, 0);
        Assert.assertEquals(new Vector2(5, 7).dot(new Vector2(5, 7)), 74, 0);
        Assert.assertEquals(new Vector2(3, 6).dot(new Vector2(-3, -6)), -45, 0);
        Assert.assertEquals(new Vector2(2, 3).dot(new Vector2(-7, 4)), -2, 0);
        Assert.assertEquals(new Vector2(4, 3).dot(new Vector2(8, 9)), 59, 0);
    }

    @Test
    // Test others methods
    public void testOthersMethods() {

        // Test the method that returns the absolute vector
        Assert.assertEquals(new Vector2(4, 5).getAbsoluteVector().equals(new Vector2(4, 5)), true);
        Assert.assertEquals(new Vector2(-7, -24).getAbsoluteVector().equals(new Vector2(7, 24)), true);

        // Test the method that returns the minimal element
        Assert.assertEquals(new Vector2(6, 35).getMinAxis(), 0, 0);
        Assert.assertEquals(new Vector2(564, 45).getMinAxis(), 1, 0);
        Assert.assertEquals(new Vector2(98, 23).getMinAxis(), 1, 0);
        Assert.assertEquals(new Vector2(-53, -25).getMinAxis(), 0, 0);

        // Test the method that returns the maximal element
        Assert.assertEquals(new Vector2(6, 35).getMaxAxis(), 1, 0);
        Assert.assertEquals(new Vector2(7, 537).getMaxAxis(), 1, 0);
        Assert.assertEquals(new Vector2(98, 23).getMaxAxis(), 0, 0);
        Assert.assertEquals(new Vector2(-53, -25).getMaxAxis(), 1, 0);
    }

    @Test
    // Test the operators
    public void testOperators() {

        // Test the [] operator
        Assert.assertEquals(mVector34.operatorSquareBrackets(0), 3, 0);
        Assert.assertEquals(mVector34.operatorSquareBrackets(1), 4, 0);

        // Assignment operator
        Vector2 newVector = new Vector2(6, 4);
        newVector = new Vector2(7, 8);

        Assert.assertEquals(newVector.equals(new Vector2(7, 8)), true);

        // Equality, inequality operators
        Assert.assertEquals(new Vector2(5, 7).operatorEquals(new Vector2(5, 7)), true);
        Assert.assertEquals(new Vector2(63, 64).operatorNotEquals(new Vector2(63, 84)), true);
        Assert.assertEquals(new Vector2(63, 64).operatorNotEquals(new Vector2(12, 64)), true);

        // Addition, substraction
        Vector2 vector1 = new Vector2(6, 33);
        Vector2 vector2 = new Vector2(7, 68);
        Assert.assertEquals(Vector2.operatorAdd(new Vector2(63, 24), new Vector2(3, 4)).equals(new Vector2(66, 28)), true);
        Assert.assertEquals(Vector2.operatorSubtract(new Vector2(63, 24), new Vector2(3, 4)).equals(new Vector2(60, 20)), true);
        vector1.operatorAddEqual(new Vector2(5, 10));
        vector2.operatorSubtractEqual(new Vector2(10, 21));

        Assert.assertEquals(vector1.operatorEquals(new Vector2(11, 43)), true);
        Assert.assertEquals(vector2.operatorEquals(new Vector2(-3, 47)), true);

        // Multiplication, division
        Assert.assertEquals(Vector2.operatorMultiply(new Vector2(63, 24), 3).equals(new Vector2(189, 72)), true);
        Assert.assertEquals(Vector2.operatorMultiply(3, new Vector2(63, 24)).equals(new Vector2(189, 72)), true);
        Assert.assertEquals(Vector2.operatorDivide(new Vector2(14, 8), 2).equals(new Vector2(7, 4)), true);

        Vector2 vector3 = new Vector2(6, 33);
        Vector2 vector4 = new Vector2(15, 60);
        vector3.operatorMultiplyEqual(10);
        vector4.operatorDivideEqual(3);

        Assert.assertEquals(vector3.equals(new Vector2(60, 330)), true);
        Assert.assertEquals(vector4.equals(new Vector2(5, 20)), true);

        // Negative operator
        Vector2 vector5 = new Vector2(-34, 5);
        Vector2 negative = Vector2.operatorNegative(vector5);

        Assert.assertEquals(negative.equals(new Vector2(34, -5)), true);
    }

}
