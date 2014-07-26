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

    // Identity transform
    private Transform mIdentityTransform;

    // First example transform
    private Transform mTransform1;

    // Second example transform
    private Transform mTransform2;

    @Before
    public void beforeEachTest() {
        mIdentityTransform = new Transform();
        mIdentityTransform.setToIdentity();

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

    @Test
    // Test the constructors
    public void testConstructors() {

        Transform transform1 = new Transform(new Vector3(1, 2, 3), new Quaternion(6, 7, 8, 9));

        Assert.assertEquals(transform1.getPosition().equals(new Vector3(1, 2, 3)), true);
        Assert.assertEquals(transform1.getOrientation().equals(new Quaternion(6, 7, 8, 9)), true);

        Transform transform2 = new Transform(new Vector3(4, 5, 6), new Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1));

        Assert.assertEquals(transform2.getPosition().equals(new Vector3(4, 5, 6)), true);
        Assert.assertEquals(transform2.getOrientation().equals(Quaternion.identity()), true);

        Transform transform3 = new Transform(transform1);

        Assert.assertEquals(transform3.equals(transform1), true);
    }

    @Test
    // Test getter and setter
    public void testGetSet() {

        Assert.assertEquals(mIdentityTransform.getPosition().equals(new Vector3(0, 0, 0)), true);
        Assert.assertEquals(mIdentityTransform.getOrientation().equals(Quaternion.identity()), true);

        Transform transform = new Transform();
        transform.setPosition(new Vector3(5, 7, 8));
        transform.setOrientation(new Quaternion(1, 2, 3, 1));

        Assert.assertEquals(transform.getPosition().equals(new Vector3(5, 7, 8)), true);
        Assert.assertEquals(transform.getOrientation().equals(new Quaternion(1, 2, 3, 1)), true);

        transform.setToIdentity();

        Assert.assertEquals(transform.getPosition().equals(new Vector3(0, 0, 0)), true);
        Assert.assertEquals(transform.getOrientation().equals(Quaternion.identity()), true);
    }

    @Test
    // Test the inverse
    public void testInverse() {

        Transform inverseTransform = mTransform1.getInverse();
        Vector3 vector = new Vector3(2, 3, 4);
        Vector3 tempVector = mTransform1.operatorMultiply(vector);
        Vector3 tempVector2 = inverseTransform.operatorMultiply(tempVector);

        Assert.assertEquals(tempVector2.x, vector.x, 10e-6f);
        Assert.assertEquals(tempVector2.y, vector.y, 10e-6f);
        Assert.assertEquals(tempVector2.z, vector.z, 10e-6f);
    }

    @Test
    // Test methods to set and get transform matrix from and to OpenGL
    public void testGetSetOpenGLMatrix() {

        Transform transform = new Transform();
        Vector3 position = mTransform1.getPosition();
        Matrix3x3 orientation = mTransform1.getOrientation().getMatrix();
        float[] openglMatrix = {orientation.m[0][0], orientation.m[1][0], orientation.m[2][0], 0,
            orientation.m[0][1], orientation.m[1][1], orientation.m[2][1], 0,
            orientation.m[0][2], orientation.m[1][2], orientation.m[2][2], 0,
            position.x, position.y, position.z, 1};
        transform.setFromOpenGL(openglMatrix);
        float[] openglMatrix2 = new float[16];
        transform.getOpenGLMatrix(openglMatrix2);

        Assert.assertEquals(openglMatrix2[0], orientation.m[0][0], 0);
        Assert.assertEquals(openglMatrix2[1], orientation.m[1][0], 0);
        Assert.assertEquals(openglMatrix2[2], orientation.m[2][0], 0);
        Assert.assertEquals(openglMatrix2[3], 0, 0);
        Assert.assertEquals(openglMatrix2[4], orientation.m[0][1], 0);
        Assert.assertEquals(openglMatrix2[5], orientation.m[1][1], 0);
        Assert.assertEquals(openglMatrix2[6], orientation.m[2][1], 0);
        Assert.assertEquals(openglMatrix2[7], 0, 0);
        Assert.assertEquals(openglMatrix2[8], orientation.m[0][2], 0);
        Assert.assertEquals(openglMatrix2[9], orientation.m[1][2], 0);
        Assert.assertEquals(openglMatrix2[10], orientation.m[2][2], 0);
        Assert.assertEquals(openglMatrix2[11], 0, 0);
        Assert.assertEquals(openglMatrix2[12], position.x, 0);
        Assert.assertEquals(openglMatrix2[13], position.y, 0);
        Assert.assertEquals(openglMatrix2[14], position.z, 0);
        Assert.assertEquals(openglMatrix2[15], 1, 0);
    }

    @Test
    // Test the method to interpolate transforms
    public void testInterpolateTransform() {

        Transform transformStart = Transform.interpolateTransforms(mTransform1, mTransform2, 0);
        Transform transformEnd = Transform.interpolateTransforms(mTransform1, mTransform2, 1);

        Assert.assertEquals(transformStart.equals(mTransform1), true);
        Assert.assertEquals(transformEnd.equals(mTransform2), true);

        float sinA = (float) Math.sin(Defaults.PI / 3.0f);
        float cosA = (float) Math.cos(Defaults.PI / 3.0f);
        float sinB = (float) Math.sin(Defaults.PI / 6.0f);
        float cosB = (float) Math.cos(Defaults.PI / 6.0f);
        Transform transform1 = new Transform(new Vector3(4, 5, 6), Quaternion.identity());
        Transform transform2 = new Transform(new Vector3(8, 11, 16), new Quaternion(sinA, sinA, sinA, cosA));
        Transform transform = Transform.interpolateTransforms(transform1, transform2, 0.5f);
        Vector3 position = transform.getPosition();
        Quaternion orientation = transform.getOrientation();

        Assert.assertEquals(position.x, 6, 0);
        Assert.assertEquals(position.y, 8, 0);
        Assert.assertEquals(position.z, 11, 0);
        Assert.assertEquals(orientation.x, sinB, 10e-6f);
        Assert.assertEquals(orientation.y, sinB, 10e-6f);
        Assert.assertEquals(orientation.z, sinB, 10e-6f);
        Assert.assertEquals(orientation.w, cosB, 10e-6f);
    }

    @Test
    // Test the identity methods
    public void testIdentity() {

        Transform transform = Transform.identity();

        Assert.assertEquals(transform.getPosition().equals(new Vector3(0, 0, 0)), true);
        Assert.assertEquals(transform.getOrientation().equals(Quaternion.identity()), true);

        Transform transform2 = new Transform(new Vector3(5, 6, 2), new Quaternion(3, 5, 1, 6));
        transform2.setToIdentity();

        Assert.assertEquals(transform2.getPosition().equals(new Vector3(0, 0, 0)), true);
        Assert.assertEquals(transform2.getOrientation().equals(Quaternion.identity()), true);
    }

    @Test
    // Test the overloaded operators
    public void testOperators() {

        // Equality, inequality operator
        Assert.assertEquals(mTransform1.operatorEquals(mTransform1), true);
        Assert.assertEquals(mTransform1.operatorNotEquals(mTransform2), true);

        // Assignment operator
        Transform transform;
        transform = mTransform1;

        Assert.assertEquals(transform.equals(mTransform1), true);

        // Multiplication
        Vector3 vector = new Vector3(7, 53, 5);
        Vector3 vector2 = mTransform2.operatorMultiply(mTransform1.operatorMultiply(vector));
        Vector3 vector3 = mTransform2.operatorMultiply(mTransform1).operatorMultiply(vector);

        Assert.assertEquals(vector2.x, vector3.x, 10e-6f);
        Assert.assertEquals(vector2.y, vector3.y, 10e-6f);
        Assert.assertEquals(vector2.z, vector3.z, 10e-6f);
    }

}
