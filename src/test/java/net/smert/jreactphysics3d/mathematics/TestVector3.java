package net.smert.jreactphysics3d.mathematics;

import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

/**
 * Unit test for the Vector3 class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestVector3 {

    @Test
    public void testClassExists() {
        Assert.assertThat(new Vector3(), CoreMatchers.instanceOf(Vector3.class));
    }

    // Test the constructors
    @Test
    public void testConstructors() {

        Vector3 test1 = new Vector3();

        Assert.assertEquals(test1.x, 0, 0);
        Assert.assertEquals(test1.y, 0, 0);
        Assert.assertEquals(test1.z, 0, 0);

        Vector3 test2 = new Vector3(3, 4, 5);

        Assert.assertEquals(test2.x, 3, 0);
        Assert.assertEquals(test2.y, 4, 0);
        Assert.assertEquals(test2.z, 5, 0);

        Vector3 vector = new Vector3(8, 15, -5);
        Vector3 test3 = new Vector3(vector);

        Assert.assertEquals(test3.x, 8, 0);
        Assert.assertEquals(test3.y, 15, 0);
        Assert.assertEquals(test3.z, -5, 0);
    }

    // Test the getter and setter methods
    @Test
    public void testGetSet() {

        // Test method to set all the values
        Vector3 test1 = new Vector3();
        test1.set(4, -4, 88);

        Assert.assertEquals(test1.x, 4, 0);
        Assert.assertEquals(test1.y, -4, 0);
        Assert.assertEquals(test1.z, 88, 0);

        // Test method to set all values to another vector
        Vector3 test2 = new Vector3(23, 42, -4);
        test1.set(test2);

        Assert.assertEquals(test1.x, 23, 0);
        Assert.assertEquals(test1.y, 42, 0);
        Assert.assertEquals(test1.z, -4, 0);

        test1.setX(10);
        test1.setY(-10);
        test1.setZ(3);

        Assert.assertEquals(test1.x, 10, 0);
        Assert.assertEquals(test1.y, -10, 0);
        Assert.assertEquals(test1.z, 3, 0);

        test1.setX(3);
        test1.setY(18);
        test1.setZ(33);

        Assert.assertEquals(test1.getX(), 3, 0);
        Assert.assertEquals(test1.getY(), 18, 0);
        Assert.assertEquals(test1.getZ(), 33, 0);
        Assert.assertEquals(test1.get(0), 3, 0);
        Assert.assertEquals(test1.get(1), 18, 0);
        Assert.assertEquals(test1.get(2), 33, 0);
    }

    // Test the length, unit vector and normalize methods
    @Test
    public void testLengthMethods() {

        Vector3 test1 = new Vector3();
        Vector3 test2 = new Vector3(1, 0, 0);
        Vector3 test3 = new Vector3(0, 1, 0);
        Vector3 test4 = new Vector3(0, 0, 1);
        Vector3 test5 = new Vector3(3, 4, 5);

        // Test length methods
        Assert.assertEquals(test1.length(), 0, 0);
        Assert.assertEquals(test1.lengthSquare(), 0, 0);
        Assert.assertEquals(test2.length(), 1, 0);
        Assert.assertEquals(test3.length(), 1, 0);
        Assert.assertEquals(test4.length(), 1, 0);
        Assert.assertEquals(test5.length(), 7.071068f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test5.lengthSquare(), 50, 0);

        // Test normalization method
        Vector3 test6 = new Vector3(5, 0, 0);
        Vector3 test7 = new Vector3(0, -5, 0);
        Vector3 test8 = new Vector3(0, 0, 5);
        test2.normalize();
        test3.normalize();
        test4.normalize();
        test6.normalize();
        test7.normalize();
        test8.normalize();

        Assert.assertEquals(test2.x, 1, 0);
        Assert.assertEquals(test2.y, 0, 0);
        Assert.assertEquals(test2.z, 0, 0);
        Assert.assertEquals(test3.x, 0, 0);
        Assert.assertEquals(test3.y, 1, 0);
        Assert.assertEquals(test3.z, 0, 0);
        Assert.assertEquals(test4.x, 0, 0);
        Assert.assertEquals(test4.y, 0, 0);
        Assert.assertEquals(test4.z, 1, 0);
        Assert.assertEquals(test6.x, 1, 0);
        Assert.assertEquals(test6.y, 0, 0);
        Assert.assertEquals(test6.z, 0, 0);
        Assert.assertEquals(test7.x, 0, 0);
        Assert.assertEquals(test7.y, -1, 0);
        Assert.assertEquals(test7.z, 0, 0);
        Assert.assertEquals(test8.x, 0, 0);
        Assert.assertEquals(test8.y, 0, 0);
        Assert.assertEquals(test8.z, 1, 0);

        // Test unit vector methods
        Assert.assertEquals(test2.isUnit(), true);
        Assert.assertEquals(test3.isUnit(), true);
        Assert.assertEquals(test4.isUnit(), true);
        Assert.assertEquals(test5.isUnit(), false);
        Assert.assertEquals(test6.isUnit(), true);
        Assert.assertEquals(test7.isUnit(), true);
        Assert.assertEquals(test8.isUnit(), true);

        // Test zero vector methods
        Assert.assertEquals(test1.isZero(), true);
        Assert.assertEquals(test5.isZero(), false);
        Assert.assertEquals(test6.isZero(), false);
        Assert.assertEquals(test7.isZero(), false);
        Assert.assertEquals(test8.isZero(), false);
    }

    // Test the dot and cross products
    @Test
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
        Vector3 test1 = new Vector3();
        Vector3 test2 = new Vector3();
        test1.cross(test2);

        Assert.assertEquals(test1.x, 0, 0);
        Assert.assertEquals(test1.y, 0, 0);
        Assert.assertEquals(test1.z, 0, 0);

        Vector3 test3 = new Vector3(6, 7, 2);
        Vector3 test4 = new Vector3(6, 7, 2);
        test3.cross(test4);

        Assert.assertEquals(test3.x, 0, 0);
        Assert.assertEquals(test3.y, 0, 0);
        Assert.assertEquals(test3.z, 0, 0);

        Vector3 test5 = new Vector3(1, 0, 0);
        Vector3 test6 = new Vector3(0, 1, 0);
        test5.cross(test6);

        Assert.assertEquals(test5.x, 0, 0);
        Assert.assertEquals(test5.y, 0, 0);
        Assert.assertEquals(test5.z, 1, 0);

        Vector3 test7 = new Vector3(0, 1, 0);
        Vector3 test8 = new Vector3(0, 0, 1);
        test7.cross(test8);

        Assert.assertEquals(test7.x, 1, 0);
        Assert.assertEquals(test7.y, 0, 0);
        Assert.assertEquals(test7.z, 0, 0);

        Vector3 test9 = new Vector3(0, 0, 1);
        Vector3 test10 = new Vector3(0, 1, 0);
        test9.cross(test10);

        Assert.assertEquals(test9.x, -1, 0);
        Assert.assertEquals(test9.y, 0, 0);
        Assert.assertEquals(test9.z, 0, 0);

        Vector3 test11 = new Vector3(4, 7, 24);
        Vector3 test12 = new Vector3(8, 13, 11);
        test11.cross(test12);

        Assert.assertEquals(test11.x, -235, 0);
        Assert.assertEquals(test11.y, 148, 0);
        Assert.assertEquals(test11.z, -4, 0);

        Vector3 test13 = new Vector3(-4, 42, -2);
        Vector3 test14 = new Vector3(35, 7, -21);
        test13.cross(test14);

        Assert.assertEquals(test13.x, -868, 0);
        Assert.assertEquals(test13.y, -154, 0);
        Assert.assertEquals(test13.z, -1498, 0);
    }

    // Test others methods
    @Test
    public void testOthersMethods() {

        // Test the method that returns the absolute vector
        Vector3 test1 = new Vector3(4, 5, 6);
        Vector3 test2 = new Vector3(-7, -24, -12);
        test1.abs();
        test2.abs();

        Assert.assertEquals(test1.x, 4, 0);
        Assert.assertEquals(test1.y, 5, 0);
        Assert.assertEquals(test1.z, 6, 0);

        Assert.assertEquals(test2.x, 7, 0);
        Assert.assertEquals(test2.y, 24, 0);
        Assert.assertEquals(test2.z, 12, 0);

        // Test the method that returns the minimal element
        Vector3 test3 = new Vector3(6, 35, 82);
        Vector3 test4 = new Vector3(564, 45, 532);
        Vector3 test5 = new Vector3(98, 23, 3);
        Vector3 test6 = new Vector3(-53, -25, -63);

        Assert.assertEquals(test3.getMinAxis(), 0, 0);
        Assert.assertEquals(test4.getMinAxis(), 1, 0);
        Assert.assertEquals(test5.getMinAxis(), 2, 0);
        Assert.assertEquals(test6.getMinAxis(), 2, 0);

        // Test the method that returns the maximal element
        Vector3 test7 = new Vector3(6, 35, 82);
        Vector3 test8 = new Vector3(7, 533, 36);
        Vector3 test9 = new Vector3(98, 23, 3);
        Vector3 test10 = new Vector3(-53, -25, -63);

        Assert.assertEquals(test7.getMaxAxis(), 2, 0);
        Assert.assertEquals(test8.getMaxAxis(), 1, 0);
        Assert.assertEquals(test9.getMaxAxis(), 0, 0);
        Assert.assertEquals(test10.getMaxAxis(), 1, 0);

        // Test zero
        test1.zero();

        Assert.assertEquals(test1.x, 0, 0);
        Assert.assertEquals(test1.y, 0, 0);
        Assert.assertEquals(test1.z, 0, 0);

        test2.setUnitOrthogonal();
        test3.setUnitOrthogonal();
        test4.setUnitOrthogonal();
        test5.setUnitOrthogonal();
        test6.setUnitOrthogonal();

        Assert.assertEquals(test2.x, 0, 0);
        Assert.assertEquals(test2.y, -0.4472136f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test2.z, 0.8944272f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test3.x, 0, 0);
        Assert.assertEquals(test3.y, -0.9197242f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test3.z, 0.3925652f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test4.x, -0.6861689f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test4.y, 0, 0);
        Assert.assertEquals(test4.z, 0.7274422f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test5.x, -0.2284856f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test5.y, 0.9735473f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test5.z, 0, 0);
        Assert.assertEquals(test6.x, 0.765226f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.y, 0, 0);
        Assert.assertEquals(test6.z, -0.6437616f, TestDefaults.FLOAT_EPSILON);

        // Test equality
        Vector3 test11 = new Vector3(test7);
        Vector3 test12 = new Vector3(test8);
        Vector3 test13 = new Vector3(test9);
        Vector3 test14 = new Vector3(test10);

        Assert.assertEquals(test7.equals(test11), true);
        Assert.assertEquals(test8.equals(test12), true);
        Assert.assertEquals(test9.equals(test13), true);
        Assert.assertEquals(test10.equals(test14), true);
        Assert.assertEquals(test10.equals(test11), false);
        Assert.assertEquals(test9.equals(test12), false);
        Assert.assertEquals(test8.equals(test13), false);
        Assert.assertEquals(test7.equals(test14), false);
    }

    // Test the operators
    @Test
    public void testOperators() {

        Vector3 vector1 = new Vector3(63, 24, 5);
        Vector3 vector2 = new Vector3(3, 4, 2);

        // Test addition
        Vector3 addition = new Vector3(vector1);
        addition.add(vector2);

        Assert.assertEquals(addition.equals(new Vector3(66, 28, 7)), true);

        // Test substraction
        Vector3 subtraction = new Vector3(vector1);
        subtraction.subtract(vector2);

        Assert.assertEquals(subtraction.equals(new Vector3(60, 20, 3)), true);

        // Test negative operator
        Vector3 negative = new Vector3(vector1);
        negative.invert();

        Assert.assertEquals(negative.equals(new Vector3(-63, -24, -5)), true);

        // Test multiplication with a number
        Vector3 multiplication1 = new Vector3(vector1);
        multiplication1.multiply(3);
        Vector3 multiplication2 = new Vector3(6, 33, 62);
        multiplication2.multiply(10);

        Assert.assertEquals(multiplication1.equals(new Vector3(189, 72, 15)), true);
        Assert.assertEquals(multiplication2.equals(new Vector3(60, 330, 620)), true);

        // Test division with a number
        Vector3 division1 = new Vector3(14, 8, 50);
        division1.divide(2);
        Vector3 division2 = new Vector3(15, 60, 33);
        division2.divide(3);

        Assert.assertEquals(division1.equals(new Vector3(7, 4, 25)), true);
        Assert.assertEquals(division2.equals(new Vector3(5, 20, 11)), true);
    }

}
