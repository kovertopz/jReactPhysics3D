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

import net.smert.jreactphysics3d.configuration.Defaults;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

/**
 * Unit test for the Quaternion class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestQuaternion {

    // First test quaternion
    private Quaternion testQuat;

    // First test vector
    private Vector3 testVector;

    @Before
    public void beforeEachTest() {
        float sinA = (float) Math.sin(Defaults.PI / 8.0);
        float cosA = (float) Math.cos(Defaults.PI / 8.0);
        testVector = new Vector3(2, 3, 4);
        testVector.normalize();
        testQuat = new Quaternion(testVector.x * sinA, testVector.y * sinA, testVector.z * sinA, cosA);
        testQuat.normalize();
    }

    @Test
    public void testClassExists() {
        Assert.assertThat(new Quaternion(), CoreMatchers.instanceOf(Quaternion.class));
    }

    // Test the constructors
    @Test
    public void testConstructors() {

        Quaternion test1 = new Quaternion();

        Assert.assertEquals(test1.w, 0, 0);
        Assert.assertEquals(test1.x, 0, 0);
        Assert.assertEquals(test1.y, 0, 0);
        Assert.assertEquals(test1.z, 0, 0);

        Quaternion test2 = new Quaternion(4, 5, 6, 7);

        Assert.assertEquals(test2.w, 7, 0);
        Assert.assertEquals(test2.x, 4, 0);
        Assert.assertEquals(test2.y, 5, 0);
        Assert.assertEquals(test2.z, 6, 0);

        Matrix3x3 matrix = new Matrix3x3();
        testQuat.getMatrix(matrix);
        Quaternion test3 = new Quaternion(matrix);

        Assert.assertEquals(test3.w, 0.9238795f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test3.x, 0.14212507f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test3.y, 0.21318759f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test3.z, 0.28425014f, TestDefaults.FLOAT_EPSILON);

        Quaternion test4 = new Quaternion(testQuat);

        Assert.assertEquals(test4.w, 0.9238795f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test4.x, 0.14212507f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test4.y, 0.21318759f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test4.z, 0.28425014f, TestDefaults.FLOAT_EPSILON);

        Quaternion test5 = new Quaternion(new Vector3(3, 5, 2), 8);

        Assert.assertEquals(test5.w, 8, 0);
        Assert.assertEquals(test5.x, 3, 0);
        Assert.assertEquals(test5.y, 5, 0);
        Assert.assertEquals(test5.z, 2, 0);
    }

    // Test the getter and setter methods
    @Test
    public void testGetSet() {

        // Test method to set all the values
        Quaternion test1 = new Quaternion();
        test1.set(4, -4, 88, 24);

        Assert.assertEquals(test1.w, 24, 0);
        Assert.assertEquals(test1.x, 4, 0);
        Assert.assertEquals(test1.y, -4, 0);
        Assert.assertEquals(test1.z, 88, 0);

        // Test method to set all values to another vector
        Quaternion test2 = new Quaternion(23, 42, -4, 16);
        test1.set(test2);

        Assert.assertEquals(test1.w, 16, 0);
        Assert.assertEquals(test1.x, 23, 0);
        Assert.assertEquals(test1.y, 42, 0);
        Assert.assertEquals(test1.z, -4, 0);

        test1.setW(-3);
        test1.setX(10);
        test1.setY(-10);
        test1.setZ(3);

        Assert.assertEquals(test1.w, -3, 0);
        Assert.assertEquals(test1.x, 10, 0);
        Assert.assertEquals(test1.y, -10, 0);
        Assert.assertEquals(test1.z, 3, 0);

        test1.setW(99);
        test1.setX(3);
        test1.setY(18);
        test1.setZ(33);

        Assert.assertEquals(test1.getW(), 99, 0);
        Assert.assertEquals(test1.getX(), 3, 0);
        Assert.assertEquals(test1.getY(), 18, 0);
        Assert.assertEquals(test1.getZ(), 33, 0);
    }

    // Test the length, unit and normalize methods
    @Test
    public void testLengthMethods() {

        Quaternion test1 = new Quaternion();
        Quaternion test2 = new Quaternion(1, 0, 0, 0);
        Quaternion test3 = new Quaternion(0, 1, 0, 0);
        Quaternion test4 = new Quaternion(0, 0, 1, 0);
        Quaternion test5 = new Quaternion(0, 0, 0, 1);
        Quaternion test6 = new Quaternion(2, 3, -4, 5);

        // Test length methods
        Assert.assertEquals(test1.length(), 0, 0);
        Assert.assertEquals(test1.lengthSquare(), 0, 0);
        Assert.assertEquals(test2.length(), 1, 0);
        Assert.assertEquals(test3.length(), 1, 0);
        Assert.assertEquals(test4.length(), 1, 0);
        Assert.assertEquals(test5.length(), 1, 0);
        Assert.assertEquals(test6.length(), 7.348469f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test6.lengthSquare(), 54, 0);

        // Test normalization method
        Quaternion test7 = new Quaternion(5, 0, 0, 0);
        Quaternion test8 = new Quaternion(0, -5, 0, 0);
        Quaternion test9 = new Quaternion(0, 0, 5, 0);
        Quaternion test10 = new Quaternion(0, 0, 0, -5);
        Quaternion test11 = new Quaternion(4, 5, 6, 7);
        test2.normalize();
        test3.normalize();
        test4.normalize();
        test5.normalize();
        test7.normalize();
        test8.normalize();
        test9.normalize();
        test10.normalize();
        test11.normalize();

        Assert.assertEquals(test2.w, 0, 0);
        Assert.assertEquals(test2.x, 1, 0);
        Assert.assertEquals(test2.y, 0, 0);
        Assert.assertEquals(test2.z, 0, 0);

        Assert.assertEquals(test3.w, 0, 0);
        Assert.assertEquals(test3.x, 0, 0);
        Assert.assertEquals(test3.y, 1, 0);
        Assert.assertEquals(test3.z, 0, 0);

        Assert.assertEquals(test4.w, 0, 0);
        Assert.assertEquals(test4.x, 0, 0);
        Assert.assertEquals(test4.y, 0, 0);
        Assert.assertEquals(test4.z, 1, 0);

        Assert.assertEquals(test5.w, 1, 0);
        Assert.assertEquals(test5.x, 0, 0);
        Assert.assertEquals(test5.y, 0, 0);
        Assert.assertEquals(test5.z, 0, 0);

        Assert.assertEquals(test7.w, 0, 0);
        Assert.assertEquals(test7.x, 1, 0);
        Assert.assertEquals(test7.y, 0, 0);
        Assert.assertEquals(test7.z, 0, 0);

        Assert.assertEquals(test8.w, 0, 0);
        Assert.assertEquals(test8.x, 0, 0);
        Assert.assertEquals(test8.y, -1, 0);
        Assert.assertEquals(test8.z, 0, 0);

        Assert.assertEquals(test9.w, 0, 0);
        Assert.assertEquals(test9.x, 0, 0);
        Assert.assertEquals(test9.y, 0, 0);
        Assert.assertEquals(test9.z, 1, 0);

        Assert.assertEquals(test10.w, -1, 0);
        Assert.assertEquals(test10.x, 0, 0);
        Assert.assertEquals(test10.y, 0, 0);
        Assert.assertEquals(test10.z, 0, 0);

        Assert.assertEquals(test11.w, 0.6236096f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test11.x, 0.35634834f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test11.y, 0.4454354f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test11.z, 0.53452253f, TestDefaults.FLOAT_EPSILON);
    }

    // Test the dot product
    @Test
    public void testDotProduct() {

        // Test the dot product
        Assert.assertEquals(new Quaternion(5, 0, 0, 0).dot(new Quaternion(0, 8, 0, 0)), 0, 0);
        Assert.assertEquals(new Quaternion(0, 5, 0, 0).dot(new Quaternion(0, 0, 8, 0)), 0, 0);
        Assert.assertEquals(new Quaternion(0, 0, 5, 0).dot(new Quaternion(0, 0, 0, 8)), 0, 0);
        Assert.assertEquals(new Quaternion(0, 0, 0, 5).dot(new Quaternion(8, 0, 0, 0)), 0, 0);
        Assert.assertEquals(new Quaternion(5, 8, 4, 5).dot(new Quaternion(0, 0, 0, 0)), 0, 0);
        Assert.assertEquals(new Quaternion(12, 45, -3, 3).dot(new Quaternion(0, 0, 0, 0)), 0, 0);
        Assert.assertEquals(new Quaternion(5, 7, 5, 7).dot(new Quaternion(5, 7, 5, 7)), 148, 0);
        Assert.assertEquals(new Quaternion(3, 6, 3, 6).dot(new Quaternion(-3, -6, 3, 6)), 0, 0);
        Assert.assertEquals(new Quaternion(2, 3, 2, 3).dot(new Quaternion(-7, 4, -7, 4)), -4, 0);
        Assert.assertEquals(new Quaternion(4, 3, 4, 3).dot(new Quaternion(8, 9, 8, 9)), 118, 0);
        Assert.assertEquals(new Quaternion(2, 3, 4, 5).dot(new Quaternion(6, 7, 8, 9)), 110, 0);
    }

    // Test others methods
    @Test
    public void testOthersMethods() {

        Quaternion test1 = new Quaternion(1, 2, 3, 4);

        // Test method to set to identity
        test1.identity();

        Assert.assertEquals(test1.w, 1, 0);
        Assert.assertEquals(test1.x, 0, 0);
        Assert.assertEquals(test1.y, 0, 0);
        Assert.assertEquals(test1.z, 0, 0);

        Quaternion test2 = new Quaternion(1, 2, 3, 4);

        // Test zero
        test2.zero();

        Assert.assertEquals(test2.w, 0, 0);
        Assert.assertEquals(test2.x, 0, 0);
        Assert.assertEquals(test2.y, 0, 0);
        Assert.assertEquals(test2.z, 0, 0);

        // Test the inverse methods
        Quaternion test3 = new Quaternion(testQuat);
        test3.inverse();

        Assert.assertEquals(test3.w, 0.9238795f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test3.x, -0.14212507f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test3.y, -0.21318759f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test3.z, -0.28425014f, TestDefaults.FLOAT_EPSILON);

        Quaternion test4 = new Quaternion(testQuat);
        test4.multiply(test3);

        Assert.assertEquals(test4.w, 1, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test4.x, 0, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test4.y, 0, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test4.z, 0, TestDefaults.FLOAT_EPSILON);

        // Test the conjugate method
        Quaternion test5 = new Quaternion(testQuat);
        test5.conjugate();

        Assert.assertEquals(test5.x, -testQuat.x, 0);
        Assert.assertEquals(test5.y, -testQuat.y, 0);
        Assert.assertEquals(test5.z, -testQuat.z, 0);
        Assert.assertEquals(test5.w, testQuat.w, 0);

        // Test the method to get the vector (x, y, z)
        Vector3 v = new Vector3();
        testQuat.getVectorV(v);

        Assert.assertEquals(v.x, testQuat.x, 0);
        Assert.assertEquals(v.y, testQuat.y, 0);
        Assert.assertEquals(v.z, testQuat.z, 0);

        // Test the method that returns the rotation angle and axis
        Vector3 axis = new Vector3();
        float[] angle = new float[1];
        testQuat.getRotationAngleAxis(axis, angle);

        Assert.assertEquals(axis.x, testVector.x, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(axis.y, testVector.y, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(axis.z, testVector.z, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(angle[0], (float) Math.PI / 4.0f, TestDefaults.FLOAT_EPSILON);

        // Test the method that returns the corresponding matrix
        Matrix3x3 matrix = testQuat.getMatrix(new Matrix3x3());
        Vector3 vector = new Vector3(56, -2, 82);
        Vector3 vector1 = matrix.multiply(vector, new Vector3());
        Vector3 vector2 = testQuat.multiply(vector, new Vector3());

        Assert.assertEquals(vector1.x, vector2.x, 10e-5); // Raise tolerance
        Assert.assertEquals(vector1.y, vector2.y, 10e-5); // Raise tolerance
        Assert.assertEquals(vector1.z, vector2.z, 10e-5); // Raise tolerance

        // Test equality
        Quaternion test6 = new Quaternion(test1);
        Quaternion test7 = new Quaternion(test2);
        Quaternion test8 = new Quaternion(test3);
        Quaternion test9 = new Quaternion(test4);

        Assert.assertEquals(test6.equals(test1), true);
        Assert.assertEquals(test7.equals(test2), true);
        Assert.assertEquals(test8.equals(test3), true);
        Assert.assertEquals(test9.equals(test4), true);
        Assert.assertEquals(test9.equals(test2), false);
        Assert.assertEquals(test8.equals(test2), false);
        Assert.assertEquals(test7.equals(test3), false);
        Assert.assertEquals(test6.equals(test2), false);

        // Test slerp method
        Quaternion quatStart = new Quaternion(2, 3, 4, 5);
        quatStart.normalize();
        Quaternion quatEnd = new Quaternion(6, 7, 8, 9);
        quatEnd.normalize();
        Quaternion test10 = new Quaternion();
        Quaternion test11 = new Quaternion();
        Quaternion.Slerp(quatStart, quatEnd, 0, test10);
        Quaternion.Slerp(quatStart, quatEnd, 1, test11);

        Quaternion quatIdent = new Quaternion(0, 0, 0, 1);
        quatIdent.normalize();
        float sinA = (float) Math.sin(Defaults.PI / 4.0f);
        float cosA = (float) Math.cos(Defaults.PI / 4.0f);
        Quaternion quat = new Quaternion(sinA, sinA, sinA, cosA);
        Quaternion test12 = new Quaternion();
        Quaternion.Slerp(quatIdent, quat, 0.5f, test12);

        Assert.assertEquals(test10.w, 0.68041384f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test10.x, 0.27216554f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test10.y, 0.4082483f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test10.z, 0.5443311f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test11.w, 0.59344244f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test11.x, 0.39562827f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test11.y, 0.46156633f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test11.z, 0.5275044f, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test12.x, (float) Math.sin(Defaults.PI / 8.0), TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test12.y, (float) Math.sin(Defaults.PI / 8.0), TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test12.z, (float) Math.sin(Defaults.PI / 8.0), TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(test12.w, (float) Math.cos(Defaults.PI / 8.0), TestDefaults.FLOAT_EPSILON);
    }

    // Test the operators
    @Test
    public void testOperators() {

        Quaternion quat1 = new Quaternion(4, 5, 2, 10);
        Quaternion quat2 = new Quaternion(-2, 7, 8, 3);
        Quaternion quat3 = new Quaternion(0, 0, 0, 1);

        // Test addition
        Quaternion addition1 = new Quaternion(quat1);
        Quaternion addition2 = new Quaternion(-6, 52, 2, 8);
        addition1.add(quat2);
        addition2.add(quat1);

        Assert.assertEquals(addition1.equals(new Quaternion(2, 12, 10, 13)), true);
        Assert.assertEquals(addition2.equals(new Quaternion(-2, 57, 4, 18)), true);

        // Test substraction
        Quaternion substraction1 = new Quaternion(quat1);
        Quaternion substraction2 = new Quaternion(-73, 62, 25, 9);
        substraction1.subtract(quat2);
        substraction2.subtract(quat1);

        Assert.assertEquals(substraction1.equals(new Quaternion(6, -2, -6, 7)), true);
        Assert.assertEquals(substraction2.equals(new Quaternion(-77, 57, 23, -1)), true);

        // Test multiplication with a number
        Quaternion multiplication1 = new Quaternion(quat1);
        multiplication1.multiply(3);

        Assert.assertEquals(multiplication1.equals(new Quaternion(12, 15, 6, 30)), true);

        // Test multiplication between two quaternions
        Quaternion multiplication2 = new Quaternion(quat1);
        Quaternion multiplication3 = new Quaternion(testQuat);
        multiplication2.multiply(quat2);
        multiplication3.multiply(quat3);

        Assert.assertEquals(multiplication2.equals(new Quaternion(18, 49, 124, -13)), true);
        Assert.assertEquals(multiplication3.equals(testQuat), true);

        // Test multiplication between a quaternion and a vector
        Matrix3x3 matrix = testQuat.getMatrix(new Matrix3x3());
        Vector3 vector = new Vector3(5, -24, 563);
        Vector3 vector1 = quat3.multiply(vector, new Vector3());
        Vector3 vector2 = testQuat.multiply(vector, new Vector3());
        Vector3 vector3 = matrix.multiply(vector, new Vector3());

        Assert.assertEquals(vector1.equals(vector), true);
        Assert.assertEquals(vector2.x, vector3.x, 10e-5); // Raise tolerance
        Assert.assertEquals(vector2.y, vector3.y, 10e-5); // Raise tolerance
        Assert.assertEquals(vector2.z, vector3.z, 10e-5); // Raise tolerance
    }

}
