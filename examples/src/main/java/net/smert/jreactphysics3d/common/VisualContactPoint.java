package net.smert.jreactphysics3d.common;

import net.smert.jreactphysics3d.common.openglframework.Mesh;
import net.smert.jreactphysics3d.common.openglframework.MeshReaderWriter;
import net.smert.jreactphysics3d.common.openglframework.Object3D;
import net.smert.jreactphysics3d.common.openglframework.Shader;
import net.smert.jreactphysics3d.common.openglframework.maths.Matrix3;
import net.smert.jreactphysics3d.common.openglframework.maths.Matrix4;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;
import org.lwjgl.opengl.GL11;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class VisualContactPoint extends Object3D {

    public static final float VISUAL_CONTACT_POINT_RADIUS = 0.1f;

    /// Total number of existing contact points (static attribute)
    private static int mNbTotalPoints = 0;

    /// Sphere mesh for the visual contact point
    private static Mesh mMesh = new Mesh();

    /// True if the mesh has been initialized
    private static boolean mIsMeshInitialized = false;

    // Constructor
    public VisualContactPoint(Vector3 position) {

        assert (mIsMeshInitialized);

        // Initialize the position where the sphere will be rendered
        translateWorld(position);
    }

    // Load and initialize the mesh for all the contact points
    public static void createStaticData() {

        if (!mIsMeshInitialized) {

            // Load the mesh from a file
            MeshReaderWriter.loadMeshFromFile("../meshes/sphere.obj", mMesh);

            // Calculate the normals of the mesh
            mMesh.calculateNormals();

            mMesh.scaleVertices(VISUAL_CONTACT_POINT_RADIUS);

            mIsMeshInitialized = true;
        }
    }

    // Destroy the mesh for the contact points
    public void destroyStaticData() {

        mMesh.destroy();
        mIsMeshInitialized = false;
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
        Matrix3 normalMatrix
                = localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
        shader.setMatrix3x3Uniform("normalMatrix", normalMatrix);

        GL11.glEnableClientState(GL11.GL_VERTEX_ARRAY);
        GL11.glEnableClientState(GL11.GL_NORMAL_ARRAY);
        if (mMesh.hasTexture()) {
            GL11.glEnableClientState(GL11.GL_TEXTURE_COORD_ARRAY);
        }

        GL11.glVertexPointer(3, GL11.GL_FLOAT, mMesh.getVerticesPointer());
        GL11.glNormalPointer(GL11.GL_FLOAT, mMesh.getNormalsPointer());
        if (mMesh.hasTexture()) {
            GL11.glTexCoordPointer(2, GL11.GL_FLOAT, mMesh.getUVTextureCoordinatesPointer());
        }

        // For each part of the mesh
        for (int i = 0; i < mMesh.getNbParts(); i++) {
            GL11.glDrawElements(GL11.GL_TRIANGLES, mMesh.getIndicesPointer());
        }

        GL11.glDisableClientState(GL11.GL_NORMAL_ARRAY);
        GL11.glDisableClientState(GL11.GL_VERTEX_ARRAY);
        if (mMesh.hasTexture()) {
            GL11.glDisableClientState(GL11.GL_TEXTURE_COORD_ARRAY);
        }

        // Unbind the shader
        shader.unbind();
    }

}
