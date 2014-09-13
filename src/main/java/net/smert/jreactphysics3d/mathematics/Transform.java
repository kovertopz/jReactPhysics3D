package net.smert.jreactphysics3d.mathematics;

import java.util.Objects;

/**
 * This class represents a position and an orientation in 3D. It can also be seen as representing a translation and a
 * rotation.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Transform {

    // Orientation
    private final Quaternion orientation;

    // Position
    private final Vector3 position;

    // Constructor
    public Transform() {
        orientation = new Quaternion().identity();
        position = new Vector3();
    }

    // Constructor with arguments
    public Transform(Vector3 position, Matrix3x3 orientation) {
        this.orientation = new Quaternion(orientation);
        this.position = new Vector3(position);
    }

    // Constructor with arguments
    public Transform(Vector3 position, Quaternion orientation) {
        this.orientation = new Quaternion(orientation);
        this.position = new Vector3(position);
    }

    // Copy-constructor
    public Transform(Transform transform) {
        orientation = new Quaternion(transform.orientation);
        position = new Vector3(transform.position);
    }

    // Return the rotation matrix
    public Quaternion getOrientation() {
        return orientation;
    }

    // Return the position of the transform
    public Vector3 getPosition() {
        return position;
    }

    // Set the transform from an OpenGL transform matrix
    public Transform fromOpenGL(float[] openglMatrix) {
        Matrix3x3 matrix = new Matrix3x3(
                openglMatrix[0], openglMatrix[4], openglMatrix[8],
                openglMatrix[1], openglMatrix[5], openglMatrix[9],
                openglMatrix[2], openglMatrix[6], openglMatrix[10]);
        orientation.fromMatrix(matrix);
        position.set(openglMatrix[12], openglMatrix[13], openglMatrix[14]);
        return this;
    }

    // Set the transform to the identity transform
    public Transform identity() {
        position.zero();
        orientation.identity();
        return this;
    }

    // Return the inverse of the transform
    public Transform inverse() {
        orientation.inverse();
        position.invert();
        Matrix3x3 invMatrix = orientation.getMatrix(new Matrix3x3());
        position.set(Matrix3x3.operatorMultiply(invMatrix, position));
        return this;
    }

    // Operator of multiplication of a transform with another one
    public Transform multiply(Transform transform) {
        Matrix3x3 matrix = orientation.getMatrix(new Matrix3x3());
        orientation.multiply(transform.orientation);
        position.add(Matrix3x3.operatorMultiply(matrix, transform.position));
        return this;
    }

    // Assignment operator
    public Transform set(Transform transform) {
        assert (transform != this);
        orientation.set(transform.orientation);
        position.set(transform.position);
        return this;
    }

    // Set the rotation matrix of the transform
    public Transform setOrientation(Quaternion orientation) {
        assert (orientation != this.orientation);
        this.orientation.set(orientation);
        return this;
    }

    // Set the origin of the transform
    public Transform setPosition(Vector3 position) {
        assert (position != this.position);
        this.position.set(position);
        return this;
    }

    // Get the OpenGL matrix of the transform
    public float[] getOpenGLMatrix(float[] openglMatrix) {
        Matrix3x3 matrix = new Matrix3x3();
        orientation.getMatrix(matrix);
        openglMatrix[0] = matrix.m00;
        openglMatrix[1] = matrix.m10;
        openglMatrix[2] = matrix.m20;
        openglMatrix[3] = 0.0f;
        openglMatrix[4] = matrix.m01;
        openglMatrix[5] = matrix.m11;
        openglMatrix[6] = matrix.m21;
        openglMatrix[7] = 0.0f;
        openglMatrix[8] = matrix.m02;
        openglMatrix[9] = matrix.m12;
        openglMatrix[10] = matrix.m22;
        openglMatrix[11] = 0.0f;
        openglMatrix[12] = position.x;
        openglMatrix[13] = position.y;
        openglMatrix[14] = position.z;
        openglMatrix[15] = 1.0f;
        return openglMatrix;
    }

    // Return the transformed vector
    public Vector3 multiply(Vector3 vector) {
        Matrix3x3 matrix = orientation.getMatrix(new Matrix3x3());
        return Matrix3x3.operatorMultiply(matrix, vector).add(position);
    }

    // Return an interpolated transform
    public static Transform Interpolate(Transform oldTransform, Transform newTransform, float interpolationFactor) {
        Quaternion interOrientation = new Quaternion();
        Vector3 interPosition = new Vector3();
        Quaternion.Slerp(oldTransform.orientation, newTransform.orientation, interpolationFactor, interOrientation);
        Vector3.Lerp(oldTransform.position, newTransform.position, interpolationFactor, interPosition);
        return new Transform(interPosition, interOrientation);
    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 71 * hash + Objects.hashCode(this.position);
        hash = 71 * hash + Objects.hashCode(this.orientation);
        return hash;
    }

    @Override
    public boolean equals(Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Transform other = (Transform) obj;
        if (!Objects.equals(this.position, other.position)) {
            return false;
        }
        return Objects.equals(this.orientation, other.orientation);
    }

    @Override
    public String toString() {
        return "(position= " + position + ", orientation= " + orientation + ")";
    }

}
