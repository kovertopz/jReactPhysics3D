package net.smert.jreactphysics3d.common.openglframework;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL14;
import org.lwjgl.opengl.GL30;
import org.lwjgl.opengl.GLContext;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class FrameBufferObject {

    // Frame buffer ID
    private int mFrameBufferID;

    // Render buffer ID
    private int mRenderBufferID;

    // Bind the FBO
    public void bind(int position) {
        assert (mFrameBufferID != 0);
        GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, mFrameBufferID);
        GL11.glDrawBuffer(position);
        GL11.glReadBuffer(position);
    }

    // Unbind the FBO
    public void unbind() {
        assert (mFrameBufferID != 0);
        GL11.glDrawBuffer(GL11.GL_NONE);
        GL11.glReadBuffer(GL11.GL_NONE);
        GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, 0);
    }

    // Return true if the needed OpenGL extensions are available for FBO
    public boolean checkOpenGLExtensions() {

        // Check that OpenGL version is at least 3.0 or there the framebuffer object extension exists
        return (GLContext.getCapabilities().OpenGL30 || GLContext.getCapabilities().GL_ARB_framebuffer_object);
    }

    // Constructor
    public FrameBufferObject() {
        mFrameBufferID = 0;
        mRenderBufferID = 0;
    }

    // Create the frame buffer object
    public boolean create(int width, int height, boolean needRenderBuffer) {

        // Destroy the current FBO
        destroy();

        // Check that the needed OpenGL extensions are available
        boolean isExtensionOK = checkOpenGLExtensions();
        if (!isExtensionOK) {
            System.out.println("Error : Impossible to use Framebuffer Object on this platform");
            assert (false);
            return false;
        }

        // Generate a new FBO
        mFrameBufferID = GL30.glGenFramebuffers();
        assert (mFrameBufferID != 0);

        // If we also need to create a render buffer
        if (needRenderBuffer) {

            // Generate the render buffer
            mRenderBufferID = GL30.glGenRenderbuffers();
            assert (mRenderBufferID != 0);

            GL30.glBindRenderbuffer(GL30.GL_RENDERBUFFER, mRenderBufferID);
            GL30.glRenderbufferStorage(GL30.GL_RENDERBUFFER, GL14.GL_DEPTH_COMPONENT32, width, height);
            GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, mFrameBufferID);
            GL30.glFramebufferRenderbuffer(GL30.GL_FRAMEBUFFER, GL30.GL_DEPTH_ATTACHMENT, GL30.GL_RENDERBUFFER, mRenderBufferID);
            GL30.glBindRenderbuffer(GL30.GL_RENDERBUFFER, 0);
            GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, 0);
        }

        // Check the FBO status
        int statusFBO = GL30.glCheckFramebufferStatus(GL30.GL_FRAMEBUFFER);
        if (statusFBO != GL30.GL_FRAMEBUFFER_COMPLETE) {
            System.out.println("Error : An error occured while creating the Frame Buffer Object !");
            assert (false);
            return false;
        }

        return true;
    }

    // Destroy the FBO
    public void destroy() {

        // Delete the frame buffer object
        if (mFrameBufferID != 0) {
            GL30.glDeleteFramebuffers(mFrameBufferID);
            mFrameBufferID = 0;
        }

        // Delete the render buffer
        if (mRenderBufferID != 0) {
            GL30.glDeleteRenderbuffers(mRenderBufferID);
        }
    }

    // Attach a texture to the frame buffer object
    public void attachTexture(int position, int textureID) {
        assert (mFrameBufferID != 0);

        // Bind the current FBO
        GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, mFrameBufferID);

        // Bind the texture
        GL30.glFramebufferTexture2D(GL30.GL_FRAMEBUFFER, position, GL11.GL_TEXTURE_2D, textureID, 0);

        // Unbind the current FBO
        GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, 0);
    }

}
