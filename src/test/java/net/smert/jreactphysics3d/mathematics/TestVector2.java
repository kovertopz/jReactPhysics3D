/*
 * ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/
 * Copyright (c) 2010-2013 Daniel Chappuis
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim
 *    that you wrote the original software. If you use this software in a
 *    product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * This file has been modified during the port to Java and differ from the source versions.
 */
package net.smert.jreactphysics3d.mathematics;

import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

/**
 * Unit test for the Vector2 class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestVector2 {

    @Test
    public void testClassExists() {
        Assert.assertThat(new Vector2(), CoreMatchers.instanceOf(Vector2.class));
    }

    // Test the constructors
    @Test
    public void testConstructors() {

        Vector2 test1 = new Vector2();

        Assert.assertEquals(test1.x, 0, 0);
        Assert.assertEquals(test1.y, 0, 0);

        Vector2 test2 = new Vector2(3, 4);

        Assert.assertEquals(test2.x, 3, 0);
        Assert.assertEquals(test2.y, 4, 0);

        Vector2 vector = new Vector2(-4, 4);
        Vector2 test3 = new Vector2(vector);

        Assert.assertEquals(test3.x, -4, 0);
        Assert.assertEquals(test3.y, 4, 0);
    }

    // Test the getter and setter methods
    @Test
    public void testGetSet() {

        // Test method to set all the values
        Vector2 test1 = new Vector2();
        test1.set(4, -4);

        Assert.assertEquals(test1.x, 4, 0);
        Assert.assertEquals(test1.y, -4, 0);

        // Test method to set all values to another vector
        Vector2 test2 = new Vector2(23, 42);
        test1.set(test2);

        Assert.assertEquals(test1.x, 23, 0);
        Assert.assertEquals(test1.y, 42, 0);

        test1.setX(10);
        test1.setY(-10);

        Assert.assertEquals(test1.x, 10, 0);
        Assert.assertEquals(test1.y, -10, 0);

        test1.setX(3);
        test1.setY(18);

        Assert.assertEquals(test1.getX(), 3, 0);
        Assert.assertEquals(test1.getY(), 18, 0);
        Assert.assertEquals(test1.get(0), 3, 0);
        Assert.assertEquals(test1.get(1), 18, 0);
    }

    // Test the length, unit vector and normalize methods
    @Test
    public void testLengthMethods() {

        Vector2 test1 = new Vector2();
        Vector2 test2 = new Vector2(1, 0);
        Vector2 test3 = new Vector2(0, 1);
        Vector2 test4 = new Vector2(3, 4);

        // Test length methods
        Assert.assertEquals(test1.length(), 0, 0);
        Assert.assertEquals(test1.lengthSquare(), 0, 0);
        Assert.assertEquals(test2.length(), 1, 0);
        Assert.assertEquals(test3.length(), 1, 0);
        Assert.assertEquals(test4.length(), 5, 0);
        Assert.assertEquals(test4.lengthSquare(), 25, 0);

        // Test normalization method
        Vector2 test5 = new Vector2(5, 0);
        Vector2 test6 = new Vector2(0, -5);
        test2.normalize();
        test3.normalize();
        test5.normalize();
        test6.normalize();

        Assert.assertEquals(test2.x, 1, 0);
        Assert.assertEquals(test2.y, 0, 0);
        Assert.assertEquals(test3.x, 0, 0);
        Assert.assertEquals(test3.y, 1, 0);
        Assert.assertEquals(test5.x, 1, 0);
        Assert.assertEquals(test5.y, 0, 0);
        Assert.assertEquals(test6.x, 0, 0);
        Assert.assertEquals(test6.y, -1, 0);

        // Test unit vector methods
        Assert.assertEquals(test2.isUnit(), true);
        Assert.assertEquals(test3.isUnit(), true);
        Assert.assertEquals(test4.isUnit(), false);
        Assert.assertEquals(test5.isUnit(), true);
        Assert.assertEquals(test6.isUnit(), true);

        // Test zero vector methods
        Assert.assertEquals(test1.isZero(), true);
        Assert.assertEquals(test4.isZero(), false);
    }

    // Test the dot product
    @Test
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

    // Test others methods
    @Test
    public void testOthersMethods() {

        // Test the method that returns the absolute vector
        Vector2 test1 = new Vector2(4, 5);
        Vector2 test2 = new Vector2(-7, -24);
        test1.abs();
        test2.abs();

        Assert.assertEquals(test1.x, 4, 0);
        Assert.assertEquals(test1.y, 5, 0);
        Assert.assertEquals(test2.x, 7, 0);
        Assert.assertEquals(test2.y, 24, 0);

        // Test the method that returns the minimal element
        Vector2 test3 = new Vector2(6, 35);
        Vector2 test4 = new Vector2(564, 45);
        Vector2 test5 = new Vector2(98, 23);
        Vector2 test6 = new Vector2(-53, -25);

        Assert.assertEquals(test3.getMinAxis(), 0, 0);
        Assert.assertEquals(test4.getMinAxis(), 1, 0);
        Assert.assertEquals(test5.getMinAxis(), 1, 0);
        Assert.assertEquals(test6.getMinAxis(), 0, 0);

        // Test the method that returns the maximal element
        Vector2 test7 = new Vector2(6, 35);
        Vector2 test8 = new Vector2(7, 537);
        Vector2 test9 = new Vector2(98, 23);
        Vector2 test10 = new Vector2(-53, -25);

        Assert.assertEquals(test7.getMaxAxis(), 1, 0);
        Assert.assertEquals(test8.getMaxAxis(), 1, 0);
        Assert.assertEquals(test9.getMaxAxis(), 0, 0);
        Assert.assertEquals(test10.getMaxAxis(), 1, 0);

        // Test zero
        test1.zero();

        Assert.assertEquals(test1.x, 0, 0);
        Assert.assertEquals(test1.y, 0, 0);

        test2.setUnitOrthogonal();
        test3.setUnitOrthogonal();

        Assert.assertEquals(test2.x, -0.96f, 0);
        Assert.assertEquals(test2.y, 0.28f, 0);
        Assert.assertEquals(test3.x, -0.985622f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test3.y, 0.168963f, TestDefaults.FLOAT_EPSILON);

        // Test equality
        Vector2 test11 = new Vector2(test7);
        Vector2 test12 = new Vector2(test8);
        Vector2 test13 = new Vector2(test9);
        Vector2 test14 = new Vector2(test10);

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

        Vector2 vector1 = new Vector2(63, 24);
        Vector2 vector2 = new Vector2(3, 4);

        // Test addition
        Vector2 addition = new Vector2(vector1);
        addition.add(vector2);

        Assert.assertEquals(addition.equals(new Vector2(66, 28)), true);

        // Test substraction
        Vector2 subtraction = new Vector2(vector1);
        subtraction.subtract(vector2);

        Assert.assertEquals(subtraction.equals(new Vector2(60, 20)), true);

        // Test negative operator
        Vector2 negative = new Vector2(vector1);
        negative.invert();

        Assert.assertEquals(negative.equals(new Vector2(-63, -24)), true);

        // Test multiplication with a number
        Vector2 multiplication1 = new Vector2(vector1);
        multiplication1.multiply(3);
        Vector2 multiplication2 = new Vector2(6, 33);
        multiplication2.multiply(10);

        Assert.assertEquals(multiplication1.equals(new Vector2(189, 72)), true);
        Assert.assertEquals(multiplication2.equals(new Vector2(60, 330)), true);

        // Test division with a number
        Vector2 division1 = new Vector2(14, 8);
        division1.divide(2);
        Vector2 division2 = new Vector2(15, 60);
        division2.divide(3);

        Assert.assertEquals(division1.equals(new Vector2(7, 4)), true);
        Assert.assertEquals(division2.equals(new Vector2(5, 20)), true);
    }

}
