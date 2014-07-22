package net.smert.jreactphysics3d.mathematics;

import net.smert.jreactphysics3d.mathematics.Vector3;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

/**
 * Unit test for the Vector3 class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestVector3 {

    /// Zero vector
    private Vector3 mVectorZero;

    // Vector (3, 4, 5)
    private Vector3 mVector345;

    @Before
    public void beforeEachTest() {
        mVectorZero = new Vector3(0, 0, 0);
        mVector345 = new Vector3(3, 4, 5);
    }

    @Test
    public void testClassExists() {
        Assert.assertThat(new Vector3(), CoreMatchers.instanceOf(Vector3.class));
    }

    @Test
    /// Test the constructors, getter and setter
    public void testConstructors() {

        // Test constructor
        Assert.assertEquals(mVectorZero.x, 0, 0);
        Assert.assertEquals(mVectorZero.y, 0, 0);
        Assert.assertEquals(mVectorZero.z, 0, 0);
        Assert.assertEquals(mVector345.x, 3, 0);
        Assert.assertEquals(mVector345.y, 4, 0);
        Assert.assertEquals(mVector345.z, 5, 0);

        // Test copy-constructor
        Vector3 newVector = new Vector3(mVector345);

        Assert.assertEquals(newVector.x, 3, 0);
        Assert.assertEquals(newVector.y, 4, 0);
        Assert.assertEquals(newVector.z, 5, 0);

        // Test method to set values
        Vector3 newVector2 = new Vector3();
        newVector2.setAllValues(6.1f, 7.2f, 8.6f);

        Assert.assertEquals(newVector2.x, 6.1f, 0);
        Assert.assertEquals(newVector2.y, 7.2f, 0);
        Assert.assertEquals(newVector2.z, 8.6f, 0);

        // Test method to set to zero
        newVector2.setToZero();

        Assert.assertEquals(newVector2.equals(new Vector3(0, 0, 0)), true);
    }

    @Test
    /// Test the length, unit vector and normalize methods
    public void testLengthMethods() {

        // Test length methods
        Assert.assertEquals(mVectorZero.length(), 0, 0);
        Assert.assertEquals(mVectorZero.lengthSquare(), 0, 0);
        Assert.assertEquals(new Vector3(1, 0, 0).length(), 1, 0);
        Assert.assertEquals(new Vector3(0, 1, 0).length(), 1, 0);
        Assert.assertEquals(new Vector3(0, 0, 1).length(), 1, 0);
        Assert.assertEquals(mVector345.lengthSquare(), 50, 0);

        // Test unit vector methods
        Assert.assertEquals(new Vector3(1, 0, 0).isUnit(), true);
        Assert.assertEquals(new Vector3(0, 1, 0).isUnit(), true);
        Assert.assertEquals(new Vector3(0, 0, 1).isUnit(), true);
        Assert.assertEquals(!mVector345.isUnit(), true);
        Assert.assertEquals(new Vector3(5, 0, 0).getUnit().equals(new Vector3(1, 0, 0)), true);
        Assert.assertEquals(new Vector3(0, 5, 0).getUnit().equals(new Vector3(0, 1, 0)), true);
        Assert.assertEquals(new Vector3(0, 0, 5).getUnit().equals(new Vector3(0, 0, 1)), true);

        Assert.assertEquals(!mVector345.isZero(), true);
        Assert.assertEquals(mVectorZero.isZero(), true);

        // Test normalization method
        Vector3 mVector100 = new Vector3(1, 0, 0);
        Vector3 mVector010 = new Vector3(0, 1, 0);
        Vector3 mVector001 = new Vector3(0, 0, 1);
        Vector3 mVector500 = new Vector3(5, 0, 0);
        Vector3 mVector050 = new Vector3(0, 5, 0);
        Vector3 mVector005 = new Vector3(0, 0, 5);
        mVector100.normalize();
        mVector010.normalize();
        mVector001.normalize();
        mVector500.normalize();
        mVector050.normalize();
        mVector005.normalize();

        Assert.assertEquals(mVector100.equals(new Vector3(1, 0, 0)), true);
        Assert.assertEquals(mVector010.equals(new Vector3(0, 1, 0)), true);
        Assert.assertEquals(mVector001.equals(new Vector3(0, 0, 1)), true);
        Assert.assertEquals(mVector500.equals(new Vector3(1, 0, 0)), true);
        Assert.assertEquals(mVector050.equals(new Vector3(0, 1, 0)), true);
        Assert.assertEquals(mVector005.equals(new Vector3(0, 0, 1)), true);
    }

    @Test
    /// Test the dot and cross products
    public void testDotCrossProducts() {

        // Test the dot product
        Assert.assertEquals(new Vector3(5, 0, 0).dot(new Vector3(0, 8, 0)), 0, 0);
        Assert.assertEquals(new Vector3(5, 8, 0).dot(new Vector3(0, 0, 6)), 0, 0);
        Assert.assertEquals(new Vector3(12, 45, 83).dot(new Vector3(0, 0, 0)), 0, 0);
        Assert.assertEquals(new Vector3(5, 7, 8).dot(new Vector3(5, 7, 8)), 138, 0);
        Assert.assertEquals(new Vector3(3, 6, 78).dot(new Vector3(-3, -6, -78)), -6129, 0);
        Assert.assertEquals(new Vector3(2, 3, 5).dot(new Vector3(2, 3, 5)), 38, 0);
        Assert.assertEquals(new Vector3(4, 3, 2).dot(new Vector3(8, 9, 10)), 79, 0);

        // Test the cross product
        Assert.assertEquals(new Vector3(0, 0, 0).cross(new Vector3(0, 0, 0)).equals(new Vector3(0, 0, 0)), true);
        Assert.assertEquals(new Vector3(6, 7, 2).cross(new Vector3(6, 7, 2)).equals(new Vector3(0, 0, 0)), true);
        Assert.assertEquals(new Vector3(1, 0, 0).cross(new Vector3(0, 1, 0)).equals(new Vector3(0, 0, 1)), true);
        Assert.assertEquals(new Vector3(0, 1, 0).cross(new Vector3(0, 0, 1)).equals(new Vector3(1, 0, 0)), true);
        Assert.assertEquals(new Vector3(0, 0, 1).cross(new Vector3(0, 1, 0)).equals(new Vector3(-1, 0, 0)), true);
        Assert.assertEquals(new Vector3(4, 7, 24).cross(new Vector3(8, 13, 11)).equals(new Vector3(-235, 148, -4)), true);
        Assert.assertEquals(new Vector3(-4, 42, -2).cross(new Vector3(35, 7, -21)).equals(new Vector3(-868, -154, -1498)), true);
    }

    @Test
    /// Test others methods
    public void testOthersMethods() {

        // Test the method that returns the absolute vector
        Assert.assertEquals(new Vector3(4, 5, 6).getAbsoluteVector().equals(new Vector3(4, 5, 6)), true);
        Assert.assertEquals(new Vector3(-7, -24, -12).getAbsoluteVector().equals(new Vector3(7, 24, 12)), true);

        // Test the method that returns the minimal element
        Assert.assertEquals(new Vector3(6, 35, 82).getMinAxis(), 0, 0);
        Assert.assertEquals(new Vector3(564, 45, 532).getMinAxis(), 1, 0);
        Assert.assertEquals(new Vector3(98, 23, 3).getMinAxis(), 2, 0);
        Assert.assertEquals(new Vector3(-53, -25, -63).getMinAxis(), 2, 0);

        // Test the method that returns the maximal element
        Assert.assertEquals(new Vector3(6, 35, 82).getMaxAxis(), 2, 0);
        Assert.assertEquals(new Vector3(7, 533, 36).getMaxAxis(), 1, 0);
        Assert.assertEquals(new Vector3(98, 23, 3).getMaxAxis(), 0, 0);
        Assert.assertEquals(new Vector3(-53, -25, -63).getMaxAxis(), 1, 0);
    }

    @Test
    /// Test the operators
    public void testOperators() {

        // Test the [] operator
        Assert.assertEquals(mVector345.operatorSquareBrackets(0), 3, 0);
        Assert.assertEquals(mVector345.operatorSquareBrackets(1), 4, 0);
        Assert.assertEquals(mVector345.operatorSquareBrackets(2), 5, 0);

        // Assignment operator
        Vector3 newVector = new Vector3(6, 4, 2);
        newVector = new Vector3(7, 8, 9);

        Assert.assertEquals(newVector.equals(new Vector3(7, 8, 9)), true);

        // Equality, inequality operators
        Assert.assertEquals(new Vector3(5, 7, 3).operatorEquals(new Vector3(5, 7, 3)), true);
        Assert.assertEquals(new Vector3(63, 64, 24).operatorNotEquals(new Vector3(63, 64, 5)), true);
        Assert.assertEquals(new Vector3(63, 64, 24).operatorNotEquals(new Vector3(12, 64, 24)), true);
        Assert.assertEquals(new Vector3(63, 64, 24).operatorNotEquals(new Vector3(63, 8, 24)), true);

        // Addition, substraction
        Assert.assertEquals(Vector3.operatorAdd(new Vector3(63, 24, 5), new Vector3(3, 4, 2)).equals(new Vector3(66, 28, 7)), true);
        Assert.assertEquals(Vector3.operatorSubtract(new Vector3(63, 24, 5), new Vector3(3, 4, 2)).equals(new Vector3(60, 20, 3)), true);

        Vector3 vector1 = new Vector3(6, 33, 62);
        Vector3 vector2 = new Vector3(7, 68, 35);
        vector1.operatorAddEqual(new Vector3(5, 10, 12));
        vector2.operatorSubtractEqual(new Vector3(10, 21, 5));

        Assert.assertEquals(vector1.equals(new Vector3(11, 43, 74)), true);
        Assert.assertEquals(vector2.equals(new Vector3(-3, 47, 30)), true);

        // Multiplication, division
        Assert.assertEquals(Vector3.operatorMultiply(new Vector3(63, 24, 5), 3).equals(new Vector3(189, 72, 15)), true);
        Assert.assertEquals(Vector3.operatorMultiply(3, new Vector3(63, 24, 5)).equals(new Vector3(189, 72, 15)), true);
        Assert.assertEquals(Vector3.operatorDivide(new Vector3(14, 8, 50), 2).equals(new Vector3(7, 4, 25)), true);

        Vector3 vector3 = new Vector3(6, 33, 62);
        Vector3 vector4 = new Vector3(15, 60, 33);
        vector3.operatorMultiplyEqual(10);
        vector4.operatorDivideEqual(3);

        Assert.assertEquals(vector3.equals(new Vector3(60, 330, 620)), true);
        Assert.assertEquals(vector4.equals(new Vector3(5, 20, 11)), true);

        // Negative operator
        Vector3 vector5 = new Vector3(-34, 5, 422);
        Vector3 negative = Vector3.operatorNegative(vector5);

        Assert.assertEquals(negative.equals(new Vector3(34, -5, -422)), true);
    }

}
