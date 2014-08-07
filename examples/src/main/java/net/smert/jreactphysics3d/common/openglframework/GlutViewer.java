package net.smert.jreactphysics3d.common.openglframework;

import net.smert.jreactphysics3d.common.glew.Glew;
import net.smert.jreactphysics3d.common.glut.Glut;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector2;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;
import org.lwjgl.util.glu.GLU;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class GlutViewer {

    // Camera
    protected Camera mCamera = new Camera();

    // Center of the scene
    protected Vector3 mCenterScene;

    // Last mouse coordinates on the windows
    protected int mLastMouseX, mLastMouseY;

    // Last point computed on a sphere (for camera rotation)
    protected Vector3 mLastPointOnSphere = new Vector3();

    // True if the last point computed on a sphere (for camera rotation) is valid
    protected boolean mIsLastPointOnSphereValid;

    // State of the mouse buttons
    protected boolean[] mIsButtonDown = new boolean[10];

    // GLUT keyboard modifiers
    protected int mModifiers;

    // Initialize the GLUT library
    protected boolean initGLUT(String[] args, String windowsTitle, Vector2 windowsSize, Vector2 windowsPosition,
            boolean isMultisamplingActive) {

        // Initialize GLUT
        Glut.glutInit(args);
        int modeWithoutMultiSampling = Glut.GLUT_DOUBLE | Glut.GLUT_RGBA | Glut.GLUT_DEPTH;
        int modeWithMultiSampling = Glut.GLUT_DOUBLE | Glut.GLUT_RGBA | Glut.GLUT_DEPTH | Glut.GL_MULTISAMPLE;
        int displayMode = isMultisamplingActive ? modeWithMultiSampling : modeWithoutMultiSampling;
        Glut.glutInitDisplayMode(displayMode);

        // Initialize the size of the GLUT windows
        Glut.glutInitWindowSize((int) windowsSize.x, (int) windowsSize.y);

        // Initialize the position of the GLUT windows
        Glut.glutInitWindowPosition((int) windowsPosition.x, (int) windowsPosition.y);

        // Create the GLUT windows
        Glut.glutCreateWindow(windowsTitle);

        // Initialize the GLEW library
        int error = Glew.glewInit();
        if (error != Glew.GLEW_OK) {

            // Problem: glewInit failed, something is wrong
            System.out.println("GLEW Error : " + Glew.glewGetErrorString(error));
            assert (false);
            return false;
        }

        return true;
    }

    // Map the mouse x,y coordinates to a point on a sphere
    protected boolean mapMouseCoordinatesToSphere(int xMouse, int yMouse, Vector3 spherePoint) {

        int width = mCamera.getWidth();
        int height = mCamera.getHeight();

        if ((xMouse >= 0) && (xMouse <= width) && (yMouse >= 0) && (yMouse <= height)) {
            float x = (xMouse - 0.5f * width) / width;
            float y = (0.5f * height - yMouse) / height;
            float sinx = (float) Math.sin(Math.PI * x * 0.5f);
            float siny = (float) Math.sin(Math.PI * y * 0.5f);
            float sinx2siny2 = sinx * sinx + siny * siny;

            // Compute the point on the sphere
            spherePoint.x = sinx;
            spherePoint.y = siny;
            spherePoint.z = (sinx2siny2 < 1.0) ? (float) Math.sqrt(1.0f - sinx2siny2) : 0.0f;

            return true;
        }

        return false;
    }

    // Set the dimension of the camera viewport
    public void reshape(int width, int height) {
        mCamera.setDimensions(width, height);
        GL11.glViewport(0, 0, width, height);
        Glut.glutPostRedisplay();
    }

    // Set the scene position (where the camera needs to look at)
    public void setScenePosition(Vector3 position, float sceneRadius) {

        // Set the position and radius of the scene
        mCenterScene = position;
        mCamera.setSceneRadius(sceneRadius);

        // Reset the camera position and zoom in order to view all the scene
        resetCameraToViewAll();
    }

    // Get the camera
    public Camera getCamera() {
        return mCamera;
    }

    // Enable/Disable the multi-sampling for anti-aliasing
    public void activateMultiSampling(boolean isActive) {
        if (isActive) {
            GL11.glEnable(GL13.GL_MULTISAMPLE);
        } else {
            GL11.glDisable(GL13.GL_MULTISAMPLE);
        }
    }

    // Constructor
    public GlutViewer() {

        // Initialize the state of the mouse buttons
        for (int i = 0; i < 10; i++) {
            mIsButtonDown[i] = false;
        }
    }

    // Initialize the viewer
    public boolean init(String[] args, String windowsTitle,
            Vector2 windowsSize, Vector2 windowsPosition) {
        return init(args, windowsTitle, windowsSize, windowsPosition, false);
    }

    // Initialize the viewer
    public boolean init(String[] args, String windowsTitle,
            Vector2 windowsSize, Vector2 windowsPosition,
            boolean isMultisamplingActive) {

        // Initialize the GLUT library
        boolean outputValue = initGLUT(args, windowsTitle, windowsSize,
                windowsPosition, isMultisamplingActive);

        // Active the multi-sampling by default
        if (isMultisamplingActive) {
            activateMultiSampling(true);
        }

        return outputValue;
    }

    // Set the camera so that we can view the whole scene
    public void resetCameraToViewAll() {

        // Move the camera to the origin of the scene
        mCamera.translateWorld(mCamera.getOrigin().operatorNegative());

        // Move the camera to the center of the scene
        mCamera.translateWorld(mCenterScene);

        // Set the zoom of the camera so that the scene center is
        // in negative view direction of the camera
        mCamera.setZoom(1.0f);
    }

    // Called when a GLUT mouse button event occurs
    public void mouseButtonEvent(int button, int state, int x, int y) {

        // If the mouse button is pressed
        if (state == Glut.GLUT_DOWN) {
            mLastMouseX = x;
            mLastMouseY = y;
            mIsLastPointOnSphereValid = mapMouseCoordinatesToSphere(x, y, mLastPointOnSphere);
            mIsButtonDown[button] = true;
        } else {  // If the mouse button is released
            mIsLastPointOnSphereValid = false;
            mIsButtonDown[button] = false;

            // If it is a mouse wheel click event
            if (button == Glut.GLUT_WHEEL_UP) {
                zoom(0, (int) (y - 0.05f * mCamera.getWidth()));
            } else if (button == Glut.GLUT_WHEEL_DOWN) {
                zoom(0, (int) (y + 0.05f * mCamera.getHeight()));
            }
        }

        mModifiers = Glut.glutGetModifiers();

        // Notify GLUT to redisplay
        Glut.glutPostRedisplay();
    }

    // Called when a GLUT mouse motion event occurs
    public void mouseMotionEvent(int xMouse, int yMouse) {

        // Zoom
        if ((mIsButtonDown[Glut.GLUT_LEFT_BUTTON] && mIsButtonDown[Glut.GLUT_MIDDLE_BUTTON])
                || (mIsButtonDown[Glut.GLUT_LEFT_BUTTON] && mModifiers == Glut.GLUT_ACTIVE_ALT)) {
            zoom(xMouse, yMouse);
        } // Translation
        else if (mIsButtonDown[Glut.GLUT_MIDDLE_BUTTON] || mIsButtonDown[Glut.GLUT_RIGHT_BUTTON]
                || (mIsButtonDown[Glut.GLUT_LEFT_BUTTON] && (mModifiers == Glut.GLUT_ACTIVE_ALT))) {
            translate(xMouse, yMouse);
        } // Rotation
        else if (mIsButtonDown[Glut.GLUT_LEFT_BUTTON]) {
            rotate(xMouse, yMouse);
        }

        // Remember the mouse position
        mLastMouseX = xMouse;
        mLastMouseY = yMouse;
        mIsLastPointOnSphereValid = mapMouseCoordinatesToSphere(xMouse, yMouse, mLastPointOnSphere);

        // Notify GLUT to redisplay
        Glut.glutPostRedisplay();
    }

    // Called when a GLUT keyboard event occurs
    public void keyboardEvent(int key, int xMouse, int yMouse) {
    }

    // Called when a GLUT special keyboard event occurs
    public void keyboardSpecialEvent(int key, int xMouse, int yMouse) {
    }

    // Zoom the camera
    public void zoom(int xMouse, int yMouse) {
        float dy = (float) (yMouse - mLastMouseY);
        float h = (float) (mCamera.getHeight());

        // Zoom the camera
        mCamera.setZoom(-dy / h);
    }

    // Translate the camera
    public void translate(int xMouse, int yMouse) {
        float dx = (float) (xMouse - mLastMouseX);
        float dy = (float) (yMouse - mLastMouseY);

        // Translate the camera
        mCamera.translateCamera(-dx / mCamera.getWidth(),
                -dy / mCamera.getHeight(), mCenterScene);
    }

    // Rotate the camera
    public void rotate(int xMouse, int yMouse) {

        if (mIsLastPointOnSphereValid) {

            Vector3 newPoint3D = new Vector3();
            boolean isNewPointOK = mapMouseCoordinatesToSphere(xMouse, yMouse, newPoint3D);

            if (isNewPointOK) {
                Vector3 axis = mLastPointOnSphere.cross(newPoint3D);
                float cosAngle = mLastPointOnSphere.dot(newPoint3D);

                float epsilon = 0.000001f;
                if (Math.abs(cosAngle) < 1.0f && axis.length() > epsilon) {
                    axis.normalize();
                    float angle = 2.0f * (float) Math.acos(cosAngle);

                    // Rotate the camera around the center of the scene
                    mCamera.rotateAroundLocalPoint(axis, -angle, mCenterScene);
                }
            }
        }
    }

    // Check the OpenGL errors
    public static void checkOpenGLErrors() {
        int glError;

        // Get the OpenGL errors
        glError = GL11.glGetError();

        // While there are errors
        while (glError != GL11.GL_NO_ERROR) {

            // Get the error string
            String stringError = GLU.gluErrorString(glError);

            // Display the error
            if (stringError != null) {
                System.out.println("OpenGL Error #" + glError + " (" + stringError + ")");
            } else {
                System.out.println("OpenGL Error #" + glError + " (no message available)");
            }

            // Get the next error
            glError = GL11.glGetError();
        }
    }

}
