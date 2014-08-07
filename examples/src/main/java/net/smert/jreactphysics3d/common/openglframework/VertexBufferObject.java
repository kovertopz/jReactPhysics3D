package net.smert.jreactphysics3d.common.openglframework;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GLContext;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class VertexBufferObject {

    /// ID of the Vertex Buffer Object
    private int mVertexBufferID;

    /// Target data. This variable must be GL_ARRAY_BUFFER if the VBO contains vertex
    /// data (vertex coordinates, texture coordinates, normals, colors) or must be
    /// GL_ELEMENT_ARRAY_BUFFER if the VBO contains index data (index array).
    private int mTargetData;

    // Bind the VBO
    public void bind() {
        assert (mVertexBufferID != 0);

        // Bind the VBO
        GL15.glBindBuffer(mTargetData, mVertexBufferID);
    }

    // Unbind the VBO
    public void unbind() {
        assert (mVertexBufferID != 0);

        // Unbind the VBO
        GL15.glBindBuffer(mTargetData, 0);
    }

    // Return true if the needed OpenGL extensions are available for VBO
    public boolean checkOpenGLExtensions() {

        // Check that OpenGL version is at least 1.5 or there the vertex buffer object extension exists
        return (GLContext.getCapabilities().OpenGL15 || GLContext.getCapabilities().GL_ARB_vertex_buffer_object);
    }

    // Constructor
    public VertexBufferObject(int targetData) {
        mVertexBufferID = 0;
        mTargetData = targetData;
    }

    // Create the vertex buffer object
    public boolean create() {

        // Destroy the current VBO
        destroy();

        // Check that the needed OpenGL extensions are available
        boolean isExtensionOK = checkOpenGLExtensions();
        if (!isExtensionOK) {
            System.out.println("Error : Impossible to use Vertex Buffer Object on this platform");
            assert (false);
            return false;
        }

        // Generate a new VBO
        mVertexBufferID = GL15.glGenBuffers();
        assert (mVertexBufferID != 0);

        return true;
    }

    // Copy data into the VBO
    public void copyDataIntoVBO(FloatBuffer data, int usage) {

        // Bind the VBO
        bind();

        // Copy the data into the VBO
        GL15.glBufferData(mTargetData, data, usage);

        // Unbind the VBO
        unbind();
    }

    // Copy data into the VBO
    public void copyDataIntoVBO(IntBuffer data, int usage) {

        // Bind the VBO
        bind();

        // Copy the data into the VBO
        GL15.glBufferData(mTargetData, data, usage);

        // Unbind the VBO
        unbind();
    }

    // Destroy the VBO
    public void destroy() {

        // Delete the vertex buffer object
        if (mVertexBufferID != 0) {
            GL15.glDeleteBuffers(mVertexBufferID);
            mVertexBufferID = 0;
        }
    }

}
