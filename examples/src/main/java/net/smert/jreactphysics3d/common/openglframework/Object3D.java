package net.smert.jreactphysics3d.common.openglframework;

import net.smert.jreactphysics3d.common.openglframework.maths.Matrix4;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;

/**
 * This class represent a generic 3D object on the scene.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Object3D {

    // Transformation matrix that convert local-space
    // coordinates to world-space coordinates
    protected Matrix4 mTransformMatrix = new Matrix4();

    // Constructor
    public Object3D() {
        setToIdentity();
    }

    // Return the transform matrix
    public Matrix4 getTransformMatrix() {
        return mTransformMatrix;
    }

    // Set the transform matrix
    public void setTransformMatrix(Matrix4 matrix) {
        mTransformMatrix = matrix;
    }

    // Set to the identity transform
    public void setToIdentity() {
        mTransformMatrix.setToIdentity();
    }

    // Return the origin of object in world-space
    public Vector3 getOrigin() {
        return mTransformMatrix.operatorMultiply(new Vector3(0.0f, 0.0f, 0.0f));
    }

    // Translate the object in world-space
    public void translateWorld(Vector3 v) {
        mTransformMatrix = Matrix4.translationMatrix(v).operatorMultiply(mTransformMatrix);
    }

    // Translate the object in local-space
    public void translateLocal(Vector3 v) {
        mTransformMatrix = mTransformMatrix.operatorMultiply(Matrix4.translationMatrix(v));
    }

    // Rotate the object in world-space
    public void rotateWorld(Vector3 axis, float angle) {
        mTransformMatrix = Matrix4.rotationMatrix(axis, angle).operatorMultiply(mTransformMatrix);
    }

    // Rotate the object in local-space
    public void rotateLocal(Vector3 axis, float angle) {
        mTransformMatrix = mTransformMatrix.operatorMultiply(Matrix4.rotationMatrix(axis, angle));
    }

    // Rotate the object around a world-space point
    public void rotateAroundWorldPoint(Vector3 axis, float angle, Vector3 worldPoint) {
        mTransformMatrix = Matrix4.translationMatrix(worldPoint).operatorMultiply(Matrix4.rotationMatrix(axis, angle)
                .operatorMultiply(Matrix4.translationMatrix(worldPoint.operatorNegative()).operatorMultiply(mTransformMatrix)));
    }

    // Rotate the object around a local-space point
    public void rotateAroundLocalPoint(Vector3 axis, float angle, Vector3 worldPoint) {

        // Convert the world point into the local coordinate system
        Vector3 localPoint = mTransformMatrix.getInverse().operatorMultiply(worldPoint);

        mTransformMatrix = mTransformMatrix.operatorMultiply(Matrix4.translationMatrix(localPoint).operatorMultiply(
                Matrix4.rotationMatrix(axis, angle).operatorMultiply(Matrix4.translationMatrix(localPoint.operatorNegative()))));
    }

}
