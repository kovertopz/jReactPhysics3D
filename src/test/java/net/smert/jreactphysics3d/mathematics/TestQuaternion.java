package net.smert.jreactphysics3d.mathematics;

import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Vector3;
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

    /// Identity Quaternion
    private Quaternion mIdentity;

    /// First test quaternion
    private Quaternion mQuaternion1;

    @Before
    public void beforeEachTest() {
        mIdentity = Quaternion.identity();

        float sinA = (float) Math.sin(Defaults.PI / 8.0);
        float cosA = (float) Math.cos(Defaults.PI / 8.0);
        Vector3 vector = new Vector3(2, 3, 4);
        vector.normalize();
        mQuaternion1 = new Quaternion(vector.x * sinA, vector.y * sinA, vector.z * sinA, cosA);
        mQuaternion1.normalize();
    }

    @Test
    public void testClassExists() {
        Assert.assertThat(new Quaternion(), CoreMatchers.instanceOf(Quaternion.class));
    }

    @Test
    /// Test the constructors
    public void testConstructors() {

        Quaternion quaternion1 = new Quaternion(mQuaternion1);

        Assert.assertEquals(quaternion1.equals(mQuaternion1), true);

        Quaternion quaternion2 = new Quaternion(4, 5, 6, 7);

        Assert.assertEquals(quaternion2.equals(new Quaternion(4, 5, 6, 7)), true);

        Quaternion quaternion3 = new Quaternion(8, new Vector3(3, 5, 2));

        Assert.assertEquals(quaternion3.equals(new Quaternion(3, 5, 2, 8)), true);

        Quaternion quaternion4 = new Quaternion(mQuaternion1.getMatrix());

        Assert.assertEquals(quaternion4.x, mQuaternion1.x, 0);
        Assert.assertEquals(quaternion4.y, mQuaternion1.y, 0);
        Assert.assertEquals(quaternion4.z, mQuaternion1.z, 0);
        Assert.assertEquals(quaternion4.w, mQuaternion1.w, 0);
    }

    @Test
    /// Test unit, length, normalize methods
    public void testUnitLengthNormalize() {

        // Test method that returns the length
        Quaternion quaternion = new Quaternion(2, 3, -4, 5);

        Assert.assertEquals(quaternion.length(), (float) Math.sqrt(54), 0);

        // Test method that returns a unit quaternion
        Assert.assertEquals(quaternion.getUnit().length(), 1, 0);

        // Test the normalization method
        Quaternion quaternion2 = new Quaternion(4, 5, 6, 7);
        quaternion2.normalize();

        Assert.assertEquals(quaternion2.length(), 1, 0);
    }

    @Test
    /// Test others methods
    public void testOthersMethods() {

        // Test the method to set the values
        Quaternion quaternion = new Quaternion();
        quaternion.setAllValues(1, 2, 3, 4);

        Assert.assertEquals(quaternion.equals(new Quaternion(1, 2, 3, 4)), true);

        // Test the method to set the quaternion to zero
        quaternion.setToZero();

        Assert.assertEquals(quaternion.equals(new Quaternion(0, 0, 0, 0)), true);

        // Tes the methods to get or set to identity
        Quaternion identity1 = new Quaternion(1, 2, 3, 4);
        identity1.setToIdentity();

        Assert.assertEquals(identity1.equals(new Quaternion(0, 0, 0, 1)), true);
        Assert.assertEquals(Quaternion.identity().equals(new Quaternion(0, 0, 0, 1)), true);

        // Test the method to get the vector (x, y, z)
        Vector3 v = mQuaternion1.getVectorV();

        Assert.assertEquals(v.x, mQuaternion1.x, 0);
        Assert.assertEquals(v.y, mQuaternion1.y, 0);
        Assert.assertEquals(v.z, mQuaternion1.z, 0);

        // Test the conjugate method
        Quaternion conjugate = mQuaternion1.getConjugate();

        Assert.assertEquals(conjugate.x, -mQuaternion1.x, 0);
        Assert.assertEquals(conjugate.y, -mQuaternion1.y, 0);
        Assert.assertEquals(conjugate.z, -mQuaternion1.z, 0);
        Assert.assertEquals(conjugate.w, mQuaternion1.w, 0);

        // Test the inverse methods
        Quaternion inverse1 = mQuaternion1.getInverse();
        Quaternion inverse2 = new Quaternion(mQuaternion1);
        inverse2.inverse();

        Assert.assertEquals(inverse1.equals(inverse2), true);

        Quaternion product = mQuaternion1.operatorMultiply(inverse1);

        Assert.assertEquals(product.x, mIdentity.x, 10e-6f);
        Assert.assertEquals(product.y, mIdentity.y, 10e-6f);
        Assert.assertEquals(product.z, mIdentity.z, 10e-6f);
        Assert.assertEquals(product.w, mIdentity.w, 10e-6f);

        // Test the dot product
        Quaternion quaternion1 = new Quaternion(2, 3, 4, 5);
        Quaternion quaternion2 = new Quaternion(6, 7, 8, 9);
        float dotProduct = quaternion1.dot(quaternion2);

        Assert.assertEquals(dotProduct, 110, 0);

        // Test the method that returns the rotation angle and axis
        Vector3 axis = new Vector3();
        float[] angle = new float[1];
        Vector3 originalAxis = new Vector3(2, 3, 4).getUnit();
        mQuaternion1.getRotationAngleAxis(angle, axis);

        Assert.assertEquals(axis.x, originalAxis.x, 10e-6f);
        Assert.assertEquals(angle[0], Defaults.PI / 4.0f, 10e-6f);

        // Test the method that returns the corresponding matrix
        Matrix3x3 matrix = mQuaternion1.getMatrix();
        Vector3 vector = new Vector3(56, -2, 82);
        Vector3 vector1 = Matrix3x3.operatorMultiply(matrix, vector);
        Vector3 vector2 = mQuaternion1.operatorMultiply(vector);

        Assert.assertEquals(vector1.x, vector2.x, 10e-6f);
        Assert.assertEquals(vector1.y, vector2.y, 10e-6f);
        Assert.assertEquals(vector1.z, vector2.z, 10e-6f);

        // Test slerp method
        Quaternion quatStart = quaternion1.getUnit();
        Quaternion quatEnd = quaternion2.getUnit();
        Quaternion test1 = Quaternion.slerp(quatStart, quatEnd, 0);
        Quaternion test2 = Quaternion.slerp(quatStart, quatEnd, 1);

        Assert.assertEquals(test1.equals(quatStart), true);
        Assert.assertEquals(test2.equals(quatEnd), true);

        float sinA = (float) Math.sin(Defaults.PI / 4.0f);
        float cosA = (float) Math.cos(Defaults.PI / 4.0f);
        Quaternion quat = new Quaternion(sinA, 0, 0, cosA);
        Quaternion test3 = Quaternion.slerp(mIdentity, quat, 0.5f);

        Assert.assertEquals(test3.x, (float) Math.sin(Defaults.PI / 8.0), 10e-6f);
        Assert.assertEquals(test3.y, 0, 0);
        Assert.assertEquals(test3.z, 0, 0);
        Assert.assertEquals(test3.w, (float) Math.cos(Defaults.PI / 8.0), 10e-6f);
    }

    @Test
    /// Test overloaded operators
    public void testOperators() {

        // Test addition
        Quaternion quat1 = new Quaternion(4, 5, 2, 10);
        Quaternion quat2 = new Quaternion(-2, 7, 8, 3);
        Quaternion test1 = quat1.operatorAdd(quat2);
        Quaternion test11 = new Quaternion(-6, 52, 2, 8);
        test11.operatorAddEqual(quat1);

        Assert.assertEquals(test1.equals(new Quaternion(2, 12, 10, 13)), true);
        Assert.assertEquals(test11.equals(new Quaternion(-2, 57, 4, 18)), true);

        // Test substraction
        Quaternion test2 = quat1.operatorSubtract(quat2);
        Quaternion test22 = new Quaternion(-73, 62, 25, 9);
        test22.operatorSubtractEqual(quat1);

        Assert.assertEquals(test2.equals(new Quaternion(6, -2, -6, 7)), true);
        Assert.assertEquals(test22.equals(new Quaternion(-77, 57, 23, -1)), true);

        // Test multiplication with a number
        Quaternion test3 = quat1.operatorMultiply(3);

        Assert.assertEquals(test3.equals(new Quaternion(12, 15, 6, 30)), true);

        // Test multiplication between two quaternions
        Quaternion test4 = quat1.operatorMultiply(quat2);
        Quaternion test5 = mQuaternion1.operatorMultiply(mIdentity);

        Assert.assertEquals(test4.equals(new Quaternion(18, 49, 124, -13)), true);
        Assert.assertEquals(test5.equals(mQuaternion1), true);

        // Test multiplication between a quaternion and a point
        Vector3 point = new Vector3(5, -24, 563);
        Vector3 vector1 = mIdentity.operatorMultiply(point);
        Vector3 vector2 = mQuaternion1.operatorMultiply(point);
        Vector3 testVector2 = Matrix3x3.operatorMultiply(mQuaternion1.getMatrix(), point);

        Assert.assertEquals(vector1.equals(point), true);
        Assert.assertEquals(vector2.x, testVector2.x, 10e-5f);
        Assert.assertEquals(vector2.y, testVector2.y, 10e-5f);
        Assert.assertEquals(vector2.z, testVector2.z, 10e-5f);

        // Test assignment operator
        Quaternion quaternion;
        quaternion = mQuaternion1;
        Assert.assertEquals(quaternion == mQuaternion1, true);

        // Test equality operator
        Assert.assertEquals(mQuaternion1.operatorEquals(mQuaternion1), true);
    }

}
