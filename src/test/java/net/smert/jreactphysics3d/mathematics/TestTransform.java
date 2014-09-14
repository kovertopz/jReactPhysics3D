package net.smert.jreactphysics3d.mathematics;

import net.smert.jreactphysics3d.configuration.Defaults;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

/**
 * Unit test for the Transform class
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestTransform {

    // First example transform
    private Transform mTransform1;

    // Second example transform
    private Transform mTransform2;

    @Before
    public void beforeEachTest() {
        float sinA = (float) Math.sin(Defaults.PI / 8.0f);
        float cosA = (float) Math.cos(Defaults.PI / 8.0f);
        mTransform1 = new Transform(new Vector3(4, 5, 6), new Quaternion(sinA, sinA, sinA, cosA));

        float sinB = (float) Math.sin(Defaults.PI / 3.0f);
        float cosB = (float) Math.cos(Defaults.PI / 3.0f);
        mTransform2 = new Transform(new Vector3(8, 45, -6), new Quaternion(sinB, sinB, sinB, cosB));
    }

    @Test
    public void testClassExists() {
        Assert.assertThat(new Transform(), CoreMatchers.instanceOf(Transform.class));
    }

    // Test the constructors
    @Test
    public void testConstructors() {

        Transform test1 = new Transform();

        Assert.assertEquals(test1.getOrientation().equals(new Quaternion(0, 0, 0, 1)), true);
        Assert.assertEquals(test1.getPosition().equals(new Vector3(0, 0, 0)), true);

        Transform test2 = new Transform(new Vector3(4, 5, 6), new Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1));

        Assert.assertEquals(test2.getOrientation().equals(new Quaternion(0, 0, 0, 1)), true);
        Assert.assertEquals(test2.getPosition().equals(new Vector3(4, 5, 6)), true);

        Transform test3 = new Transform(new Vector3(1, 2, 3), new Quaternion(6, 7, 8, 9));

        Assert.assertEquals(test3.getOrientation().equals(new Quaternion(6, 7, 8, 9)), true);
        Assert.assertEquals(test3.getPosition().equals(new Vector3(1, 2, 3)), true);

        Transform test4 = new Transform(test3);

        Assert.assertEquals(test4.equals(test3), true);
    }

    // Test getter and setter
    @Test
    public void testGetSet() {

        Transform test1 = new Transform();
        test1.setOrientation(new Quaternion(6, 7, 8, 9));
        test1.setPosition(new Vector3(1, 2, 3));

        Assert.assertEquals(test1.getOrientation().equals(new Quaternion(6, 7, 8, 9)), true);
        Assert.assertEquals(test1.getPosition().equals(new Vector3(1, 2, 3)), true);

        Transform test2 = new Transform(new Vector3(4, 5, 6), new Quaternion(4, 3, 2, 1));
        test2.set(test1);

        Assert.assertEquals(test2.getOrientation().equals(new Quaternion(6, 7, 8, 9)), true);
        Assert.assertEquals(test2.getPosition().equals(new Vector3(1, 2, 3)), true);
    }

    // Test others methods
    @Test
    public void testOthersMethods() {

        // Test the identity methods
        Transform test1 = new Transform(new Vector3(1, 2, 3), new Quaternion(6, 7, 8, 9));
        test1.identity();

        Assert.assertEquals(test1.getOrientation().equals(new Quaternion(0, 0, 0, 1)), true);
        Assert.assertEquals(test1.getPosition().equals(new Vector3(0, 0, 0)), true);

        // Test the inverse
        Transform test2 = new Transform(mTransform1);
        test2.inverse();

        Vector3 vector = new Vector3(2, 3, 4);
        Vector3 vector1 = mTransform1.multiply(vector, new Vector3());
        Vector3 vector2 = test2.multiply(vector1, new Vector3());

        Assert.assertEquals(vector2.x, vector.x, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(vector2.y, vector.y, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(vector2.z, vector.z, TestDefaults.FLOAT_EPSILON);

        // Test methods to set and get transform matrix from and to OpenGL
        Quaternion orientation = new Quaternion(6, 7, 8, 9).normalize();
        Matrix3x3 rotation = orientation.getMatrix(new Matrix3x3());
        Vector3 position = new Vector3(3, 4, 5);
        float[] fromOpenglMatrix = {rotation.m00, rotation.m10, rotation.m20, 0,
            rotation.m01, rotation.m11, rotation.m21, 0,
            rotation.m02, rotation.m12, rotation.m22, 0,
            position.x, position.y, position.z, 1};

        Transform test3 = new Transform();
        test3.fromOpenGL(fromOpenglMatrix);

        Assert.assertEquals(test3.getOrientation().equals(orientation), true);
        Assert.assertEquals(test3.getPosition().equals(position), true);

        float[] getOpenglMatrix = new float[16];
        test3.getOpenGLMatrix(getOpenglMatrix);

        Assert.assertEquals(getOpenglMatrix[0], rotation.m00, 0);
        Assert.assertEquals(getOpenglMatrix[1], rotation.m10, 0);
        Assert.assertEquals(getOpenglMatrix[2], rotation.m20, 0);
        Assert.assertEquals(getOpenglMatrix[3], 0, 0);
        Assert.assertEquals(getOpenglMatrix[4], rotation.m01, 0);
        Assert.assertEquals(getOpenglMatrix[5], rotation.m11, 0);
        Assert.assertEquals(getOpenglMatrix[6], rotation.m21, 0);
        Assert.assertEquals(getOpenglMatrix[7], 0, 0);
        Assert.assertEquals(getOpenglMatrix[8], rotation.m02, 0);
        Assert.assertEquals(getOpenglMatrix[9], rotation.m12, 0);
        Assert.assertEquals(getOpenglMatrix[10], rotation.m22, 0);
        Assert.assertEquals(getOpenglMatrix[11], 0, 0);
        Assert.assertEquals(getOpenglMatrix[12], position.x, 0);
        Assert.assertEquals(getOpenglMatrix[13], position.y, 0);
        Assert.assertEquals(getOpenglMatrix[14], position.z, 0);
        Assert.assertEquals(getOpenglMatrix[15], 1, 0);

        // Test the method to interpolate transforms
        Transform test4 = Transform.Interpolate(mTransform1, mTransform2, 0, new Transform());
        Transform test5 = Transform.Interpolate(mTransform1, mTransform2, 1, new Transform());

        Assert.assertEquals(test4.equals(mTransform1), true);
        Assert.assertEquals(test5.equals(mTransform2), true);

        float sinA = (float) Math.sin(Defaults.PI / 3.0f);
        float cosA = (float) Math.cos(Defaults.PI / 3.0f);
        float sinB = (float) Math.sin(Defaults.PI / 6.0f);
        float cosB = (float) Math.cos(Defaults.PI / 6.0f);
        Transform transform1 = new Transform(new Vector3(4, 5, 6), new Quaternion().identity());
        Transform transform2 = new Transform(new Vector3(8, 11, 16), new Quaternion(sinA, sinA, sinA, cosA));

        Transform test6 = Transform.Interpolate(transform1, transform2, 0.5f, new Transform());
        orientation = test6.getOrientation();
        position = test6.getPosition();

        Assert.assertEquals(orientation.w, cosB, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(orientation.x, sinB, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(orientation.y, sinB, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(orientation.z, sinB, TestDefaults.FLOAT_EPSILON);
        Assert.assertEquals(position.x, 6, 0);
        Assert.assertEquals(position.y, 8, 0);
        Assert.assertEquals(position.z, 11, 0);

        // Test equality
        Transform test7 = new Transform(test1);
        Transform test8 = new Transform(test2);
        Transform test9 = new Transform(test3);
        Transform test10 = new Transform(test4);

        Assert.assertEquals(test7.equals(test1), true);
        Assert.assertEquals(test8.equals(test2), true);
        Assert.assertEquals(test9.equals(test3), true);
        Assert.assertEquals(test10.equals(test4), true);
        Assert.assertEquals(mTransform1.equals(mTransform1), true);
        Assert.assertEquals(test10.equals(test1), false);
        Assert.assertEquals(test9.equals(test2), false);
        Assert.assertEquals(test8.equals(test3), false);
        Assert.assertEquals(test7.equals(test4), false);
        Assert.assertEquals(mTransform1.equals(mTransform2), false);
    }

    // Test the overloaded operators
    @Test
    public void testOperators() {

        // Test multiplication with a transform
        Transform test1 = new Transform(mTransform1);
        Transform test2 = new Transform();
        test2.identity();
        test1.multiply(test2);

        Assert.assertEquals(test1.equals(mTransform1), true);

        Transform test3 = new Transform(mTransform1);
        Transform test4 = new Transform(mTransform2);
        test4.inverse();

        test3.multiply(mTransform2).multiply(test4);
        Quaternion test3Orientation = test3.getOrientation();
        Vector3 test3Position = test3.getPosition();
        Quaternion orientation = mTransform1.getOrientation();
        Vector3 position = mTransform1.getPosition();

        Assert.assertEquals(test3Orientation.w, orientation.w, 10e-5); // Raise tolerance
        Assert.assertEquals(test3Orientation.x, orientation.x, 10e-5); // Raise tolerance
        Assert.assertEquals(test3Orientation.y, orientation.y, 10e-5); // Raise tolerance
        Assert.assertEquals(test3Orientation.z, orientation.z, 10e-5); // Raise tolerance
        Assert.assertEquals(test3Position.x, position.x, 10e-5); // Raise tolerance
        Assert.assertEquals(test3Position.y, position.y, 10e-5); // Raise tolerance
        Assert.assertEquals(test3Position.z, position.z, 10e-5); // Raise tolerance
    }

}
