package net.smert.jreactphysics3d.common;

import java.util.List;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.collision.shapes.ConvexMeshShape;
import net.smert.jreactphysics3d.common.openglframework.Mesh;
import net.smert.jreactphysics3d.common.openglframework.MeshReaderWriter;
import net.smert.jreactphysics3d.common.openglframework.Shader;
import net.smert.jreactphysics3d.common.openglframework.maths.Matrix3;
import net.smert.jreactphysics3d.common.openglframework.maths.Matrix4;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;
import net.smert.jreactphysics3d.engine.DynamicsWorld;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Transform;
import org.lwjgl.opengl.GL11;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ConvexMesh extends Mesh {

    /// Rigid body used to simulate the dynamics of the mesh
    private RigidBody mRigidBody;

    // Return a pointer to the rigid body of the mesh
    public RigidBody getRigidBody() {
        return mRigidBody;
    }

    // Constructor
    public ConvexMesh(Vector3 position, float mass, DynamicsWorld dynamicsWorld) {
        super();

        // Load the mesh from a file
        MeshReaderWriter.loadMeshFromFile("../meshes/convexmesh.obj", this);

        // Calculate the normals of the mesh
        calculateNormals();

        // Initialize the position where the sphere will be rendered
        translateWorld(position);

        // Create the collision shape for the rigid body (convex mesh shape)
        // ReactPhysics3D will clone this object to create an internal one. Therefore,
        // it is OK if this object is destroyed right after calling Dynamics::createRigidBody()
        List<Vector3> vertices = getVertices();
        float[] verticesArray = new float[vertices.size() * 3];
        for (int i = 0, j = 0; i < vertices.size(); i++) {
            Vector3 v = vertices.get(i);
            verticesArray[j++] = v.x;
            verticesArray[j++] = v.y;
            verticesArray[j++] = v.z;
        }
        ConvexMeshShape collisionShape = new ConvexMeshShape(verticesArray, verticesArray.length, 3, 0.02f);

        // Add the edges information of the mesh into the convex mesh collision shape.
        // This is optional but it really speed up the convex mesh collision detection at the
        // cost of some additional memory to store the edges inside the collision shape.
        for (int i = 0; i < getNbFaces(); i++) { // For each triangle face of the mesh

            // Get the three vertex IDs of the vertices of the face
            int v1 = getVertexIndexInFace(i, 0);
            int v2 = getVertexIndexInFace(i, 1);
            int v3 = getVertexIndexInFace(i, 2);

            // Add the three edges into the collision shape
            collisionShape.addEdge(v1, v2);
            collisionShape.addEdge(v1, v3);
            collisionShape.addEdge(v2, v3);
        }
        collisionShape.setIsEdgesInformationUsed(true);// Enable the fast collision detection with edges

        // Compute the inertia tensor of the body using its collision shape
        Matrix3x3 inertiaTensor = new Matrix3x3();
        collisionShape.computeLocalInertiaTensor(inertiaTensor, mass);

        // Initial position and orientation of the rigid body
        net.smert.jreactphysics3d.mathematics.Vector3 initPosition = new net.smert.jreactphysics3d.mathematics.Vector3(position.x, position.y, position.z);
        Quaternion initOrientation = new Quaternion().identity();
        Transform transform = new Transform(initPosition, initOrientation);

        // Create a rigid body corresponding to the sphere in the dynamics world
        mRigidBody = dynamicsWorld.createRigidBody(transform, mass, inertiaTensor, collisionShape);
    }

    // Render the sphere at the correct position and with the correct orientation
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

        GL11.glEnableClientState(GL11.GL_VERTEX_ARRAY);
        GL11.glEnableClientState(GL11.GL_NORMAL_ARRAY);
        if (hasTexture()) {
            GL11.glEnableClientState(GL11.GL_TEXTURE_COORD_ARRAY);
        }

        GL11.glVertexPointer(3, 0, getVerticesPointer());
        GL11.glNormalPointer(0, getNormalsPointer());
        if (hasTexture()) {
            GL11.glTexCoordPointer(2, 0, getUVTextureCoordinatesPointer());
        }

        // For each part of the mesh
        for (int i = 0; i < getNbParts(); i++) {
            GL11.glDrawElements(GL11.GL_TRIANGLES, getIndicesPointer());
        }

        GL11.glDisableClientState(GL11.GL_NORMAL_ARRAY);
        GL11.glDisableClientState(GL11.GL_VERTEX_ARRAY);
        if (hasTexture()) {
            GL11.glDisableClientState(GL11.GL_TEXTURE_COORD_ARRAY);
        }

        // Unbind the shader
        shader.unbind();
    }

    // Update the transform matrix of the sphere
    public void updateTransform() {

        // Get the interpolated transform of the rigid body
        Transform transform = mRigidBody.getInterpolatedTransform();

        // Compute the transform used for rendering the sphere
        float[] matrix = new float[16];
        transform.getOpenGLMatrix(matrix);
        Matrix4 newMatrix = new Matrix4(matrix[0], matrix[4], matrix[8], matrix[12],
                matrix[1], matrix[5], matrix[9], matrix[13],
                matrix[2], matrix[6], matrix[10], matrix[14],
                matrix[3], matrix[7], matrix[11], matrix[15]);

        // Apply the scaling matrix to have the correct sphere dimensions
        mTransformMatrix = newMatrix;
    }

}
