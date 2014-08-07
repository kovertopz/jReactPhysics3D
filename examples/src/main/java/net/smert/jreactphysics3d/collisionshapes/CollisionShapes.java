package net.smert.jreactphysics3d.collisionshapes;

import net.smert.jreactphysics3d.common.Viewer;
import net.smert.jreactphysics3d.common.glut.Glut;
import net.smert.jreactphysics3d.common.openglframework.GlutViewer;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector2;
import org.lwjgl.input.Keyboard;
import org.lwjgl.opengl.GL11;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CollisionShapes {

    private Scene scene;
    private final Viewer viewer;

    public CollisionShapes(String[] args) {

        // Create and initialize the Viewer
        viewer = new Viewer();
        Vector2 windowsSize = new Vector2(800, 600);
        Vector2 windowsPosition = new Vector2(100, 100);
        boolean initOK = viewer.init(args, "ReactPhysics3D Examples - Collision Shapes", windowsSize, windowsPosition);
        if (!initOK) {
            return;
        }

        // Create the scene
        scene = new Scene(viewer);

        init();

        // Glut Idle function that is continuously called
        Glut.glutIdleFunc(this, "simulate");
        Glut.glutDisplayFunc(this, "display");
        Glut.glutReshapeFunc(this, "reshape");
        Glut.glutMouseFunc(this, "mouseButton");
        Glut.glutMotionFunc(this, "mouseMotion");
        Glut.glutKeyboardFunc(this, "keyboard");

        Glut.glutCloseFunc(this, "finish");
    }

    public void run() {

        // Glut main looop
        Glut.glutMainLoop();
    }

    // Simulate function
    public void simulate() {

        // Physics simulation
        scene.simulate();

        viewer.computeFPS();

        // Ask GLUT to render the scene
        Glut.glutPostRedisplay();
    }

    // Initialization
    public void init() {

        // Define the background color (black)
        GL11.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    }

    // Reshape function
    public void reshape(int newWidth, int newHeight) {
        viewer.reshape(newWidth, newHeight);
    }

    // Called when a mouse button event occurs
    public void mouseButton(int button, int state, int x, int y) {
        viewer.mouseButtonEvent(button, state, x, y);
    }

    // Called when a mouse motion event occurs
    public void mouseMotion(int x, int y) {
        viewer.mouseMotionEvent(x, y);
    }

    // Called when the user hits a special key on the keyboard
    public void keyboard(int key, int x, int y) {
        switch (key) {

            // Escape key
            case Keyboard.KEY_ESCAPE:
                Glut.glutLeaveMainLoop();
                break;

            // Space bar
            case Keyboard.KEY_SPACE:
                scene.pauseContinueSimulation();
                break;
        }
    }

    // End of the application
    public void finish() {

        // Destroy the viewer and the scene
    }

    // Display the scene
    public void display() {

        // Render the scene
        scene.render();

        // Display the FPS
        viewer.displayGUI();

        // Swap the buffers
        Glut.glutSwapBuffers();

        // Check the OpenGL errors
        GlutViewer.checkOpenGLErrors();
    }

}
