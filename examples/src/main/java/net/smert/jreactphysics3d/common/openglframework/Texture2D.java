package net.smert.jreactphysics3d.common.openglframework;

import java.nio.FloatBuffer;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL12;
import org.lwjgl.opengl.GL13;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Texture2D {

    // OpenGL texture ID
    private int mID;

    // Layer of the texture
    private int mLayer;

    // Width
    private int mWidth;

    // Height
    private int mHeight;

    // Bind the texture
    public void bind() {
        assert (mID != 0);
        GL11.glEnable(GL11.GL_TEXTURE_2D);
        GL13.glActiveTexture(GL13.GL_TEXTURE0 + mLayer);
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, mID);
    }

    // Unbind the texture
    public void unbind() {
        assert (mID != 0);
        GL13.glActiveTexture(GL13.GL_TEXTURE0 + mLayer);
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, 0);
        GL11.glDisable(GL11.GL_TEXTURE_2D);
    }

    // Get the OpenGL texture ID
    public int getID() {
        return mID;
    }

    // Get the layer of the texture
    public int getLayer() {
        return mLayer;
    }

    // Set the layer of the texture
    public void setLayer(int layer) {
        mLayer = layer;
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
    public Texture2D() {
        mID = 0;
        mLayer = 0;
        mWidth = 0;
        mHeight = 0;
    }

    // Constructor
    public Texture2D(int width, int height, int internalFormat, int format, int type) {
        mID = 0;
        mLayer = 0;
        mWidth = width;
        mHeight = height;

        // Create the texture
        create(width, height, internalFormat, format, type, null);
    }

    public void create(int width, int height, int internalFormat, int format, int type) {
        create(width, height, internalFormat, format, type, null);
    }

    // Create the texture
    public void create(int width, int height, int internalFormat, int format, int type, FloatBuffer data) {

        // Destroy the current texture
        destroy();

        mWidth = width;
        mHeight = height;

        // Create the OpenGL texture
        mID = GL11.glGenTextures();
        assert (mID != 0);
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, mID);
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_WRAP_S, GL12.GL_CLAMP_TO_EDGE);
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_WRAP_T, GL12.GL_CLAMP_TO_EDGE);
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_LINEAR);
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_LINEAR);
        GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, internalFormat, mWidth, mHeight, 0, format, type, data);
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, 0);
    }

    // Destroy the texture
    public void destroy() {
        if (mID != 0) {
            GL11.glDeleteTextures(mID);
            mID = 0;
            mLayer = 0;
            mWidth = 0;
            mHeight = 0;
        }
    }

}
