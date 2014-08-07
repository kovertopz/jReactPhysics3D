package net.smert.jreactphysics3d.common;

import net.smert.jreactphysics3d.common.openglframework.GlutViewer;
import org.lwjgl.opengl.GL11;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Viewer extends GlutViewer {

    /// Current number of frames per seconds
    private int fps;

    /// Number of frames during the last second
    private int nbFrames;

    /// Current time for fps computation
    private int currentTime;

    /// Previous time for fps computation
    private int previousTime;

    // Constructor
    public Viewer() {
        fps = 0;
        nbFrames = 0;
    }

    // Compute the FPS
    public void computeFPS() {

        nbFrames++;

        //  Get the number of milliseconds since glutInit called
        currentTime = (int) System.nanoTime() / 1000000;

        //  Calculate time passed
        int timeInterval = currentTime - previousTime;

        // Update the FPS counter each second
        if (timeInterval > 1000) {

            //  calculate the number of frames per second
            fps = (int) (nbFrames / (timeInterval / 1000.0f));

            //  Set time
            previousTime = currentTime;

            //  Reset frame count
            nbFrames = 0;
        }
    }

    // Display the GUI
    public void displayGUI() {

        // Display the FPS
        displayFPS();
    }

    // Display the FPS
    public void displayFPS() {

        GL11.glMatrixMode(GL11.GL_PROJECTION);
        GL11.glLoadIdentity();
        GL11.glOrtho(0, mCamera.getWidth(), mCamera.getHeight(), 0, -1, 1);

        GL11.glMatrixMode(GL11.GL_MODELVIEW);
        GL11.glLoadIdentity();

        GL11.glRasterPos2i(10, 20);
        GL11.glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        //System.out.println("FPS : " + fps);
    }

}
