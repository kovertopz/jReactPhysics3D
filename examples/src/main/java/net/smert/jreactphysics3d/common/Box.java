package net.smert.jreactphysics3d.common;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.collision.shapes.BoxShape;
import net.smert.jreactphysics3d.common.openglframework.Object3D;
import net.smert.jreactphysics3d.common.openglframework.Shader;
import net.smert.jreactphysics3d.common.openglframework.VertexBufferObject;
import net.smert.jreactphysics3d.common.openglframework.maths.Color;
import net.smert.jreactphysics3d.common.openglframework.maths.Matrix3;
import net.smert.jreactphysics3d.common.openglframework.maths.Matrix4;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector4;
import net.smert.jreactphysics3d.engine.DynamicsWorld;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Transform;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Box extends Object3D {

    /// Size of each side of the box
    private float[] mSize = new float[3];

    /// Rigid body used to simulate the dynamics of the box
    private RigidBody mRigidBody;

    /// Scaling matrix (applied to a cube to obtain the correct box dimensions)
    private Matrix4 mScalingMatrix;

    /// Vertex Buffer Object for the vertices data used to render the box with OpenGL
    private static VertexBufferObject mVBOVertices = new VertexBufferObject(GL15.GL_ARRAY_BUFFER);

    /// Vertex Buffer Object for the indices used to render the box with OpenGL
    private static VertexBufferObject mVBOIndices = new VertexBufferObject(GL15.GL_ELEMENT_ARRAY_BUFFER);

    /// Vertex data for each vertex of the cube (used to render the box)
    private static float[] mCubeVertices = new float[]{
        1, 1, 1, 0, 0, 1, 1, 0, 0, 1, //V0
        -1, 1, 1, 0, 0, 1, 1, 0, 0, 1, //V1
        -1, -1, 1, 0, 0, 1, 1, 0, 0, 1, //V2
        1, -1, 1, 0, 0, 1, 1, 0, 0, 1, //V3

        1, 1, -1, 1, 0, 0, 1, 0, 0, 1, //V5
        1, 1, 1, 1, 0, 0, 1, 0, 0, 1, //V0
        1, -1, 1, 1, 0, 0, 1, 0, 0, 1, //V3
        1, -1, -1, 1, 0, 0, 1, 0, 0, 1, //V4

        -1, 1, -1, 0, 0, -1, 1, 0, 0, 1, //V6
        1, 1, -1, 0, 0, -1, 1, 0, 0, 1, //V5
        1, -1, -1, 0, 0, -1, 1, 0, 0, 1, //V4
        -1, -1, -1, 0, 0, -1, 1, 0, 0, 1, //V7

        1, 1, -1, 0, 1, 0, 1, 0, 0, 1, //V5
        -1, 1, -1, 0, 1, 0, 1, 0, 0, 1, //V6
        -1, 1, 1, 0, 1, 0, 1, 0, 0, 1, //V1
        1, 1, 1, 0, 1, 0, 1, 0, 0, 1, //V0

        -1, 1, 1, -1, 0, 0, 1, 0, 0, 1, //V1
        -1, 1, -1, -1, 0, 0, 1, 0, 0, 1, //V6
        -1, -1, -1, -1, 0, 0, 1, 0, 0, 1, //V7
        -1, -1, 1, -1, 0, 0, 1, 0, 0, 1, //V2

        -1, -1, -1, 0, -1, 0, 1, 0, 0, 1, //V7
        1, -1, -1, 0, -1, 0, 1, 0, 0, 1, //V4
        1, -1, 1, 0, -1, 0, 1, 0, 0, 1, //V3
        -1, -1, 1, 0, -1, 0, 1, 0, 0, 1, //V2
    };

    /// Indices of the cube (used to render the box)
    private static int[] mCubeIndices = new int[]{
        0, 1, 2, 3, //Front face
        4, 5, 6, 7, //Right face
        8, 9, 10, 11, //Back face
        12, 13, 14, 15, //Upper face
        16, 17, 18, 19, //Left face
        20, 21, 22, 23, //Bottom face
    };

    /// True if the VBOs have already been created
    private static boolean areVBOsCreated = false;

    /// Main color of the box
    private Color mColor;

    // Return a pointer to the rigid body of the box
    public RigidBody getRigidBody() {
        return mRigidBody;
    }

    // Set the color of the box
    public void setColor(Color color) {
        mColor = color;
    }

    // Constructor
    public Box(Vector3 size, Vector3 position, float mass, DynamicsWorld dynamicsWorld) {
        super();

        mColor = new Color(0.5f, 0.5f, 0.5f, 1);

        // Initialize the size of the box
        mSize[0] = size.x * 0.5f;
        mSize[1] = size.y * 0.5f;
        mSize[2] = size.z * 0.5f;

        // Compute the scaling matrix
        mScalingMatrix = new Matrix4(mSize[0], 0, 0, 0,
                0, mSize[1], 0, 0,
                0, 0, mSize[2], 0,
                0, 0, 0, 1);

        // Initialize the position where the cube will be rendered
        translateWorld(position);

        // Create the collision shape for the rigid body (box shape)
        // ReactPhysics3D will clone this object to create an internal one. Therefore,
        // it is OK if this object is destroyed right after calling Dynamics::createRigidBody()
        BoxShape collisionShape = new BoxShape(new net.smert.jreactphysics3d.mathematics.Vector3(mSize[0], mSize[1], mSize[2]), 0.02f);

        // Compute the inertia tensor of the body using its collision shape
        Matrix3x3 inertiaTensor = new Matrix3x3();
        collisionShape.computeLocalInertiaTensor(inertiaTensor, mass);

        // Initial position and orientation of the rigid body
        net.smert.jreactphysics3d.mathematics.Vector3 initPosition = new net.smert.jreactphysics3d.mathematics.Vector3(position.x, position.y, position.z);
        Quaternion initOrientation = new Quaternion().identity();
        Transform transform = new Transform(initPosition, initOrientation);

        // Create a rigid body corresponding to the cube in the dynamics world
        mRigidBody = dynamicsWorld.createRigidBody(transform, mass, inertiaTensor, collisionShape);

        // If the Vertex Buffer object has not been created yet
        if (!areVBOsCreated) {
            // Create the Vertex Buffer
            createVBO();
        }
    }

    // Render the cube at the correct position and with the correct orientation
    public void render(Shader shader, Matrix4 worldToCameraMatrix) {

        // Bind the shader
        shader.bind();

        // Set the model to camera matrix
        Matrix4 localToCameraMatrix = worldToCameraMatrix.operatorMultiply(mTransformMatrix);
        shader.setMatrix4x4Uniform("localToCameraMatrix", localToCameraMatrix);

        // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the
        // model-view matrix)
        Matrix3 normalMatrix = localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
        shader.setMatrix3x3Uniform("normalMatrix", normalMatrix);

        // Set the vertex color
        Vector4 color = new Vector4(mColor.r, mColor.g, mColor.b, mColor.a);
        shader.setVector4Uniform("vertexColor", color);

        // Bind the vertices VBO
        mVBOVertices.bind();

        // Enable the vertex, normal and color arrays
        GL11.glEnableClientState(GL11.GL_VERTEX_ARRAY);
        GL11.glEnableClientState(GL11.GL_NORMAL_ARRAY);
        GL11.glEnableClientState(GL11.GL_COLOR_ARRAY);

        // Set the arrays pointers
        GL11.glVertexPointer(3, GL11.GL_FLOAT, 40, 0);
        GL11.glNormalPointer(GL11.GL_FLOAT, 40, 0);
        GL11.glColorPointer(4, GL11.GL_FLOAT, 40, 0);

        // Bind the indices VBO
        mVBOIndices.bind();

        // Draw the geometry of the box
        GL11.glDrawElements(GL11.GL_QUADS, mCubeVertices.length, GL11.GL_UNSIGNED_INT, 0);

        // Unbind the VBOs
        mVBOVertices.unbind();
        mVBOIndices.unbind();

        // Disable the arrays
        GL11.glDisableClientState(GL11.GL_VERTEX_ARRAY);
        GL11.glDisableClientState(GL11.GL_NORMAL_ARRAY);
        GL11.glDisableClientState(GL11.GL_COLOR_ARRAY);

        // Unbind the shader
        shader.unbind();
    }

    // Update the transform matrix of the box
    public void updateTransform() {

        // Get the interpolated transform of the rigid body
        Transform transform = mRigidBody.getInterpolatedTransform();

        // Compute the transform used for rendering the box
        float[] matrix = new float[16];
        transform.getOpenGLMatrix(matrix);
        Matrix4 newMatrix = new Matrix4(matrix[0], matrix[4], matrix[8], matrix[12],
                matrix[1], matrix[5], matrix[9], matrix[13],
                matrix[2], matrix[6], matrix[10], matrix[14],
                matrix[3], matrix[7], matrix[11], matrix[15]);

        // Apply the scaling matrix to have the correct box dimensions
        mTransformMatrix = newMatrix.operatorMultiply(mScalingMatrix);
    }

    // Create the Vertex Buffer Objects used to render to box with OpenGL.
    /// We create two VBOs (one for vertices and one for indices) to render all the boxes
    /// in the simulation.
    public void createVBO() {

        // Create the VBOs
        mVBOVertices.create();
        mVBOIndices.create();

        FloatBuffer fb = BufferUtils.createFloatBuffer(mCubeVertices.length);
        fb.put(mCubeVertices);
        fb.flip();

        IntBuffer ib = BufferUtils.createIntBuffer(mCubeIndices.length);
        ib.put(mCubeIndices);
        ib.flip();

        // Copy the data into the VBOs
        mVBOVertices.copyDataIntoVBO(fb, GL15.GL_STATIC_DRAW);
        mVBOIndices.copyDataIntoVBO(ib, GL15.GL_STATIC_DRAW);

        areVBOsCreated = true;
    }

}
