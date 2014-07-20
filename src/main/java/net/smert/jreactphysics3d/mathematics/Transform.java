package net.smert.jreactphysics3d.mathematics;

/**
 * This class represents a position and an orientation in 3D. It can also be seen as representing a translation and a
 * rotation.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Transform {

    /// Position
    private Vector3 mPosition;

    /// Orientation
    private Quaternion mOrientation;

    // Constructor
    public Transform() {
        mPosition = new Vector3(0.0f, 0.0f, 0.0f);
        mOrientation = Quaternion.identity();
    }

    // Constructor
    public Transform(Vector3 position, Matrix3x3 orientation) {
        mPosition = position;
        mOrientation = new Quaternion(orientation);
    }

    // Constructor
    public Transform(Vector3 position, Quaternion orientation) {
        mPosition = position;
        mOrientation = orientation;
    }

    // Copy-constructor
    public Transform(Transform transform) {
        mPosition = transform.mPosition;
        mOrientation = transform.mOrientation;
    }

    // Return the position of the transform
    public Vector3 getPosition() {
        return mPosition;
    }

    // Set the origin of the transform
    public void setPosition(Vector3 position) {
        mPosition = position;
    }

    // Return the rotation matrix
    public Quaternion getOrientation() {
        return mOrientation;
    }

    // Set the rotation matrix of the transform
    public void setOrientation(Quaternion orientation) {
        mOrientation = orientation;
    }

    // Set the transform to the identity transform
    public void setToIdentity() {
        mPosition = new Vector3(0.0f, 0.0f, 0.0f);
        mOrientation = Quaternion.identity();
    }

    // Set the transform from an OpenGL transform matrix
    public void setFromOpenGL(float[] openglMatrix) {
        Matrix3x3 matrix = new Matrix3x3(openglMatrix[0], openglMatrix[4], openglMatrix[8],
                openglMatrix[1], openglMatrix[5], openglMatrix[9],
                openglMatrix[2], openglMatrix[6], openglMatrix[10]);
        mOrientation = new Quaternion(matrix);
        mPosition.setAllValues(openglMatrix[12], openglMatrix[13], openglMatrix[14]);
    }

    // Get the OpenGL matrix of the transform
    public void getOpenGLMatrix(float[] openglMatrix) {
        Matrix3x3 matrix = mOrientation.getMatrix();
        openglMatrix[0] = matrix.m[0][0];
        openglMatrix[1] = matrix.m[1][0];
        openglMatrix[2] = matrix.m[2][0];
        openglMatrix[3] = 0.0f;
        openglMatrix[4] = matrix.m[0][1];
        openglMatrix[5] = matrix.m[1][1];
        openglMatrix[6] = matrix.m[2][1];
        openglMatrix[7] = 0.0f;
        openglMatrix[8] = matrix.m[0][2];
        openglMatrix[9] = matrix.m[1][2];
        openglMatrix[10] = matrix.m[2][2];
        openglMatrix[11] = 0.0f;
        openglMatrix[12] = mPosition.x;
        openglMatrix[13] = mPosition.y;
        openglMatrix[14] = mPosition.z;
        openglMatrix[15] = 1.0f;
    }

    // Return the inverse of the transform
    public Transform getInverse() {
        Quaternion invQuaternion = mOrientation.getInverse();
        Matrix3x3 invMatrix = invQuaternion.getMatrix();
        return new Transform(Matrix3x3.operatorMultiply(invMatrix, Vector3.operatorNegative(mPosition)), invQuaternion);
    }

    // Return an interpolated transform
    public Transform interpolateTransforms(Transform oldTransform, Transform newTransform, float interpolationFactor) {

        Vector3 interPosition = Vector3.operatorMultiply(oldTransform.mPosition, 1.0f - interpolationFactor).operatorAddEqual(Vector3.operatorMultiply(newTransform.mPosition, interpolationFactor));

        Quaternion interOrientation = Quaternion.slerp(oldTransform.mOrientation,
                newTransform.mOrientation,
                interpolationFactor);

        return new Transform(interPosition, interOrientation);
    }

    // Return the identity transform
    public static Transform identity() {
        return new Transform(new Vector3(0.0f, 0.0f, 0.0f), Quaternion.identity());
    }

    // Return the transformed vector
    public Vector3 operatorMultiply(Vector3 vector) {
        Matrix3x3 matrix = mOrientation.getMatrix();
        return Matrix3x3.operatorMultiply(matrix, vector).operatorAddEqual(mPosition);
    }

    // Operator of multiplication of a transform with another one
    public Transform operatorMultiply(Transform transform2) {
        return new Transform(Vector3.operatorAdd(mPosition, Matrix3x3.operatorMultiply(mOrientation.getMatrix(), transform2.mPosition)),
                mOrientation.operatorMultiply(transform2.mOrientation));
    }

    // Return true if the two transforms are equal
    public boolean operatorEquals(Transform transform2) {
        return (mPosition == transform2.mPosition) && (mOrientation == transform2.mOrientation);
    }

    // Return true if the two transforms are different
    public boolean operatorNotEquals(Transform transform2) {
        return !(operatorEquals(transform2));
    }

    // Assignment operator
    public Transform operatorEqual(Transform transform) {
        if (transform != this) {
            mPosition = transform.mPosition;
            mOrientation = transform.mOrientation;
        }
        return this;
    }

}
