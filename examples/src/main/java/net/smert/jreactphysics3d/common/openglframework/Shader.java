package net.smert.jreactphysics3d.common.openglframework;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.nio.FloatBuffer;
import net.smert.jreactphysics3d.common.openglframework.maths.Matrix3;
import net.smert.jreactphysics3d.common.openglframework.maths.Matrix4;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector2;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector4;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GLContext;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Shader {

    // Shader object program ID
    private int mProgramObjectID;

    // Filenames of the vertex and fragment shaders
    private String mFilenameVertexShader, mFilenameFragmentShader;

    // Bind the shader
    public void bind() {
        assert (mProgramObjectID != 0);
        GL20.glUseProgram(mProgramObjectID);
    }

    // Unbind the shader
    public void unbind() {
        assert (mProgramObjectID != 0);
        GL20.glUseProgram(0);
    }

    // Return the location of a uniform variable inside a shader program
    public int getUniformLocation(String variableName) {
        assert (mProgramObjectID != 0);
        int location = GL20.glGetUniformLocation(mProgramObjectID, variableName);
        if (location == -1) {
            System.out.println("Error in vertex shader " + mFilenameVertexShader + " or in fragment shader"
                    + mFilenameFragmentShader + " : No Uniform variable : " + variableName);
        }
        assert (location != -1);
        return location;
    }

    // Clear the shader
    public void destroy() {
        if (mProgramObjectID != 0) {
            GL20.glDeleteShader(mProgramObjectID);
            mProgramObjectID = 0;
        }
    }

    // Set a float uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try
    // to set it, an assert will occur)
    public void setFloatUniform(String variableName, float value) {
        assert (mProgramObjectID != 0);
        GL20.glUniform1f(getUniformLocation(variableName), value);
    }

    // Set an int uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try
    // to set it, an assert will occur)
    public void setIntUniform(String variableName, int value) {
        assert (mProgramObjectID != 0);
        GL20.glUniform1i(getUniformLocation(variableName), value);
    }

    // Set a vector 2 uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try
    // to set it, an assert will occur)
    public void setVector2Uniform(String variableName, Vector2 v) {
        assert (mProgramObjectID != 0);
        GL20.glUniform2f(getUniformLocation(variableName), v.x, v.y);
    }

    // Set a vector 3 uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try
    // to set it, an assert will occur)
    public void setVector3Uniform(String variableName, Vector3 v) {
        assert (mProgramObjectID != 0);
        GL20.glUniform3f(getUniformLocation(variableName), v.x, v.y, v.z);
    }

    // Set a vector 4 uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try
    // to set it, an assert will occur)
    public void setVector4Uniform(String variableName, Vector4 v) {
        assert (mProgramObjectID != 0);
        GL20.glUniform4f(getUniformLocation(variableName), v.x, v.y, v.z, v.w);
    }

    // Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try
    // to set it, an assert will occur)
    public void setMatrix3x3Uniform(String variableName, FloatBuffer matrix, boolean transpose) {
        assert (mProgramObjectID != 0);
        GL20.glUniformMatrix3(getUniformLocation(variableName), transpose, matrix);
    }

    // Set a 3x3 matrix uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try
    // to set it, an assert will occur)
    public void setMatrix3x3Uniform(String variableName, Matrix3 matrix) {
        assert (mProgramObjectID != 0);
        float[] mat = new float[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                mat[i * 3 + j] = matrix.getValue(i, j);
            }
        }
        FloatBuffer fb = BufferUtils.createFloatBuffer(9);
        fb.put(mat);
        fb.flip();
        GL20.glUniformMatrix3(getUniformLocation(variableName), true, fb);
    }

    // Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try
    // to set it, an assert will occur)
    public void setMatrix4x4Uniform(String variableName, FloatBuffer matrix, boolean transpose) {
        assert (mProgramObjectID != 0);
        GL20.glUniformMatrix4(getUniformLocation(variableName), transpose, matrix);
    }

    // Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try
    // to set it, an assert will occur)
    public void setMatrix4x4Uniform(String variableName, Matrix4 matrix) {
        assert (mProgramObjectID != 0);
        float[] mat = new float[16];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                mat[i * 4 + j] = matrix.m[i][j];
            }
        }
        FloatBuffer fb = BufferUtils.createFloatBuffer(16);
        fb.put(mat);
        fb.flip();
        GL20.glUniformMatrix4(getUniformLocation(variableName), true, fb);
    }

    // Return true if the needed OpenGL extensions are available for shaders
    public boolean checkOpenGLExtensions() {

        // Check that GLSL vertex and fragment shaders are available on the platform
        return (GLContext.getCapabilities().OpenGL20 || (GLContext.getCapabilities().GL_ARB_vertex_shader
                && GLContext.getCapabilities().GL_ARB_fragment_shader));
    }

    // Constructor
    public Shader() {
        mProgramObjectID = 0;
    }

    // Constructor with arguments
    public Shader(String vertexShaderFilename, String fragmentShaderFilename) {
        mProgramObjectID = 0;

        // Create the shader
        create(vertexShaderFilename, fragmentShaderFilename);
    }

    // Create the shader
    public boolean create(String vertexShaderFilename, String fragmentShaderFilename) {

        // Set the shader filenames
        mFilenameVertexShader = vertexShaderFilename;
        mFilenameFragmentShader = fragmentShaderFilename;

        // Check that the needed OpenGL extensions are available
        boolean isExtensionOK = checkOpenGLExtensions();
        if (!isExtensionOK) {
            System.out.println("Error : Impossible to use GLSL vertex or fragment shaders on this platform");
            assert (false);
            return false;
        }

        // Delete the current shader
        destroy();

        assert (vertexShaderFilename.length() != 0 && fragmentShaderFilename.length() != 0);

        System.out.println(fragmentShaderFilename);
        System.out.println(vertexShaderFilename);
        String fragmentShaderCode = readShaderFromJAR(fragmentShaderFilename);
        String vertexShaderCode = readShaderFromJAR(vertexShaderFilename);

        mProgramObjectID = GL20.glCreateProgram();
        int fragmentShader;
        int vertexShader;

        if (mProgramObjectID != 0) {
            fragmentShader = compileShader(mFilenameFragmentShader, fragmentShaderCode, GL20.GL_FRAGMENT_SHADER);
            vertexShader = compileShader(mFilenameVertexShader, vertexShaderCode, GL20.GL_VERTEX_SHADER);
        } else {
            throw new RuntimeException("Unable to create shader program object!");
        }

        if ((fragmentShader != 0) && (vertexShader != 0)) {
            GL20.glAttachShader(mProgramObjectID, fragmentShader);
            GL20.glAttachShader(mProgramObjectID, vertexShader);

            GL20.glLinkProgram(mProgramObjectID);

            if ((GL20.glGetProgrami(mProgramObjectID, GL20.GL_LINK_STATUS) != GL11.GL_FALSE) == false) {
                System.err.println(GL20.glGetShaderInfoLog(mProgramObjectID, 8192));
                throw new RuntimeException("Shader \"" + mFilenameVertexShader + "\" had linking errors!");
            }

            if (true) {
                GL20.glValidateProgram(mProgramObjectID);

                if ((GL20.glGetProgrami(mProgramObjectID, GL20.GL_VALIDATE_STATUS) != GL11.GL_FALSE) == false) {
                    System.err.println(GL20.glGetShaderInfoLog(mProgramObjectID, 8192));
                    throw new RuntimeException("Shader \"" + mFilenameVertexShader + "\" had validate errors!");
                }
            }

            if (true) {
                System.out.println("Shader \"" + mFilenameVertexShader + "\" was successfully loaded.");
            }
        }

        return true;
    }

    protected int compileShader(String shadername, String shadercode, int shadertype) {
        final int shaderid = GL20.glCreateShader(shadertype);

        if (shaderid == 0) {
            throw new RuntimeException("Unable to create shader! " + shadertype);
        }

        GL20.glShaderSource(shaderid, shadercode);
        GL20.glCompileShader(shaderid);

        if ((GL20.glGetShaderi(shaderid, GL20.GL_COMPILE_STATUS) != GL11.GL_FALSE) == false) {
            System.err.println(GL20.glGetShaderInfoLog(shaderid, 8192));
            throw new RuntimeException("Shader \"" + shadername + "\" had compile errors!");
        }

        return shaderid;
    }

    protected String readShaderFromJAR(String file) {
        String line;
        String shadercode = "";

        try {
            InputStream inputstream = this.getClass().getResourceAsStream(file);
            BufferedReader reader = new BufferedReader(new InputStreamReader(inputstream));

            while ((line = reader.readLine()) != null) {
                shadercode += line + "\n";
            }

            reader.close();
            inputstream.close();
        } catch (IOException e) {
            throw new RuntimeException("Unable to open shader file: " + file);
        }

        return shadercode;
    }

}
