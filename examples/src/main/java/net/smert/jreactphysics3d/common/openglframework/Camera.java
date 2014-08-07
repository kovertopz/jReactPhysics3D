package net.smert.jreactphysics3d.common.openglframework;

import net.smert.jreactphysics3d.common.openglframework.maths.Matrix4;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Camera extends Object3D {

    // Field of view
    protected float mFieldOfView;

    // Radius of the scene
    protected float mSceneRadius;

    // Near plane
    protected float mNearPlane;

    // Far plane
    protected float mFarPlane;

    // Width of the camera
    protected int mWidth;

    // Height of the camera
    protected int mHeight;

    // Projection matrix
    protected Matrix4 mProjectionMatrix;

    // Update the projection matrix
    protected void updateProjectionMatrix() {

        // Compute the aspect ratio
        float aspect = mWidth / mHeight;

        float top = mNearPlane * (float) Math.tan((mFieldOfView / 2.0f) * (Math.PI / 180.0f));
        float bottom = -top;
        float left = bottom * aspect;
        float right = top * aspect;

        float fx = 2.0f * mNearPlane / (right - left);
        float fy = 2.0f * mNearPlane / (top - bottom);
        float fz = -(mFarPlane + mNearPlane) / (mFarPlane - mNearPlane);
        float fw = -2.0f * mFarPlane * mNearPlane / (mFarPlane - mNearPlane);

        // Recompute the projection matrix
        mProjectionMatrix = new Matrix4(fx, 0, 0, 0,
                0, fy, 0, 0,
                0, 0, fz, fw,
                0, 0, -1, 0);
    }

    // Get the projection matrix
    public Matrix4 getProjectionMatrix() {
        return mProjectionMatrix;
    }

    // Set the dimensions of the camera
    public void setDimensions(int width, int height) {
        mWidth = width;
        mHeight = height;
        updateProjectionMatrix();
    }

    // Get the radius of the scene the camera should capture
    public float getSceneRadius() {
        return mSceneRadius;
    }

    // Set the radius of the scene the camera should capture
    // This will update the clipping planes accordingly
    public void setSceneRadius(float radius) {
        mSceneRadius = radius;
        setClippingPlanes(0.01f * radius, 10.0f * radius);
    }

    // Set the clipping planes
    public void setClippingPlanes(float near, float far) {
        mNearPlane = near;
        mFarPlane = far;
        updateProjectionMatrix();
    }

    // Set the field of view
    public void setFieldOfView(float fov) {
        mFieldOfView = fov;
        updateProjectionMatrix();
    }

    // Set the zoom of the camera (a fraction between 0 and 1)
    public void setZoom(float fraction) {
        Vector3 zoomVector = new Vector3(0, 0, mSceneRadius * fraction * 3.0f);
        translateLocal(zoomVector);
    }

    // Get the near clipping plane
    public float getNearClippingPlane() {
        return mNearPlane;
    }

    // Get the far clipping plane
    public float getFarClippingPlane() {
        return mFarPlane;
    }

    // Get the width
    public int getWidth() {
        return mWidth;
    }

    // Get the height
    public int getHeight() {
        return mHeight;
    }

    // Constructor
    public Camera() {

        // Set default values
        mFieldOfView = 45.0f;
        mSceneRadius = 1.0f;
        mNearPlane = 0.1f;
        mFarPlane = 10.0f;
        mWidth = 1;
        mHeight = 1;

        // Update the projection matrix
        updateProjectionMatrix();
    }

    // Translate the camera go a given point using the dx, dy fraction
    public void translateCamera(float dx, float dy, Vector3 worldPoint) {

        // Transform the world point into camera coordinates
        Vector3 pointCamera = mTransformMatrix.getInverse().operatorMultiply(worldPoint);

        // Get the depth
        float z = -pointCamera.z;

        // Find the scaling of dx and dy from windows coordinates to near plane coordinates
        // and from there to camera coordinates at the object's depth
        float aspect = mWidth / mHeight;
        float top = mNearPlane * (float) Math.tan(mFieldOfView * Math.PI / 360.0f);
        float right = top * aspect;

        // Translate the camera
        translateLocal(new Vector3(2.0f * dx * right / mNearPlane * z,
                -2.0f * dy * top / mNearPlane * z,
                0.0f));
    }

}
