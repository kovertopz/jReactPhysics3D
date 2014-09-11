package net.smert.jreactphysics3d.common.openglframework;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import net.smert.jreactphysics3d.common.openglframework.maths.Color;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector2;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;
import org.lwjgl.BufferUtils;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Mesh extends Object3D {

    // A triplet of vertex indices for each triangle
    protected List<Integer> mIndices = new ArrayList<>();

    // Vertices coordinates (local space)
    protected List<Vector3> mVertices = new ArrayList<>();

    // Normals coordinates
    protected List<Vector3> mNormals = new ArrayList<>();

    // Tangents coordinates
    protected List<Vector3> mTangents = new ArrayList<>();

    // Color for each vertex
    protected List<Color> mColors = new ArrayList<>();

    // UV texture coordinates
    protected List<Vector2> mUVs = new ArrayList<>();

    // Textures of the mesh (one for each part of the mesh)
    protected Map<Integer, Texture2D> mTextures = new HashMap<>();

    // Return the number of triangles
    public int getNbFaces() {
        return mIndices.size() / 3;
    }

    // Return the number of vertices
    public int getNbVertices() {
        return mVertices.size();
    }

    // Return the number of parts in the mesh
    public int getNbParts() {
        return mIndices.size();
    }

    // Return a reference to the vertices
    public List<Vector3> getVertices() {
        return mVertices;
    }

    // Set the vertices of the mesh
    public void setVertices(List<Vector3> vertices) {
        mVertices = vertices;
    }

    // Return a reference to the normals
    public List<Vector3> getNormals() {
        return mNormals;
    }

    // set the normals of the mesh
    public void setNormals(List<Vector3> normals) {
        mNormals = normals;
    }

    // Return a reference to the UVs
    public List<Vector2> getUVs() {
        return mUVs;
    }

    // Set the UV texture coordinates of the mesh
    public void setUVs(List<Vector2> uvs) {
        mUVs = uvs;
    }

    // Return a reference to the vertex indices
    public List<Integer> getIndices() {
        return mIndices;
    }

    // Set the vertices indices of the mesh
    public void setIndices(List<Integer> indices) {
        mIndices = indices;
    }

    // Return the coordinates of a given vertex
    public Vector3 getVertex(int i) {
        assert (i < getNbVertices());
        return mVertices.get(i);
    }

    // Set the coordinates of a given vertex
    public void setVertex(int i, Vector3 vertex) {
        assert (i < getNbVertices());
        mVertices.add(i, vertex);
    }

    // Return the coordinates of a given normal
    public Vector3 getNormal(int i) {
        assert (i < getNbVertices());
        return mNormals.get(i);
    }

    // Set the coordinates of a given normal
    public void setNormal(int i, Vector3 normal) {
        assert (i < getNbVertices());
        mNormals.add(i, normal);
    }

    // Return the color of a given vertex
    public Color getColor(int i) {
        assert (i < getNbVertices());
        return mColors.get(i);
    }

    // Set the color of a given vertex
    public void setColor(int i, Color color) {

        // If the color array does not have the same size as
        // the vertices array
        if (mColors.size() != mVertices.size()) {

            // Create the color array with the same size
            mColors = new ArrayList<>(mVertices.size());
        }

        mColors.add(i, color);
    }

    // Set a color to all the vertices
    public void setColorToAllVertices(Color color) {

        // If the color array does not have the same size as
        // the vertices array
        if (mColors.size() != mVertices.size()) {

            // Create the color array with the same size
            mColors = new ArrayList<>(mVertices.size());
        }

        for (int v = 0; v < mVertices.size(); v++) {
            mColors.add(v, color);
        }
    }

    // Return the UV of a given vertex
    public Vector2 getUV(int i) {
        assert (i < getNbVertices());
        return mUVs.get(i);
    }

    // Set the UV of a given vertex
    public void setUV(int i, Vector2 uv) {
        assert (i < getNbVertices());
        mUVs.add(i, uv);
    }

    // Return the vertex index of the ith (i=0,1,2) vertex of a given face
    public int getVertexIndexInFace(int faceIndex, int i) {
        return mIndices.get(faceIndex * 3 + i);
    }

    // Return true if the mesh has normals
    public boolean hasNormals() {
        return mNormals.size() == mVertices.size();
    }

    // Return true if the mesh has tangents
    public boolean hasTangents() {
        return mTangents.size() == mVertices.size();
    }

    // Return true if the mesh has vertex colors
    public boolean hasColors() {
        return mColors.size() == mVertices.size();
    }

    // Return true if the mesh has UV texture coordinates
    public boolean hasUVTextureCoordinates() {
        return mUVs.size() == mVertices.size();
    }

    // Return true if the mesh has a texture for a given part of the mesh and if it
    // also have texture coordinates
    public boolean hasTextureForPart(int part) {
        return hasUVTextureCoordinates() && mTextures.get(part) != null;
    }

    // Return true if the mesh has a texture (and texture coordinates) for at least one
    // part of the mesh
    public boolean hasTexture() {
        return hasUVTextureCoordinates() && (mTextures.size() > 0);
    }

    private FloatBuffer verticesPointer;

    // Return a pointer to the vertices data
    public FloatBuffer getVerticesPointer() {
        if (verticesPointer == null) {
            verticesPointer = BufferUtils.createFloatBuffer(mVertices.size() * 3);
            for (Vector3 v : mVertices) {
                verticesPointer.put(v.x);
                verticesPointer.put(v.y);
                verticesPointer.put(v.z);
            }
            verticesPointer.flip();
        }

        return verticesPointer;
    }

    private FloatBuffer normalsPointer;

    // Return a pointer to the normals data
    public FloatBuffer getNormalsPointer() {
        if (normalsPointer == null) {
            normalsPointer = BufferUtils.createFloatBuffer(mNormals.size() * 3);
            for (Vector3 v : mNormals) {
                normalsPointer.put(v.x);
                normalsPointer.put(v.y);
                normalsPointer.put(v.z);
            }
            normalsPointer.flip();
        }

        return normalsPointer;
    }

    private ByteBuffer colorsPointer;

    // Return a pointer to the colors data
    public ByteBuffer getColorsPointer() {
        if (colorsPointer == null) {
            colorsPointer = BufferUtils.createByteBuffer(mColors.size() * 4);
            for (Color c : mColors) {
                colorsPointer.put((byte) c.r);
                colorsPointer.put((byte) c.g);
                colorsPointer.put((byte) c.b);
                colorsPointer.put((byte) c.a);
            }
            colorsPointer.flip();
        }

        return colorsPointer;
    }

    private FloatBuffer tangentsPointer;

    // Return a pointer to the tangents data
    public FloatBuffer getTangentsPointer() {
        if (tangentsPointer == null) {
            tangentsPointer = BufferUtils.createFloatBuffer(mTangents.size() * 3);
            for (Vector3 v : mTangents) {
                tangentsPointer.put(v.x);
                tangentsPointer.put(v.y);
                tangentsPointer.put(v.z);
            }
            tangentsPointer.flip();
        }

        return tangentsPointer;
    }

    private FloatBuffer textureCoordinatesPointer;

    // Return a pointer to the UV texture coordinates data
    public FloatBuffer getUVTextureCoordinatesPointer() {
        if (textureCoordinatesPointer == null) {
            textureCoordinatesPointer = BufferUtils.createFloatBuffer(mUVs.size() * 2);
            for (Vector2 v : mUVs) {
                textureCoordinatesPointer.put(v.x);
                textureCoordinatesPointer.put(v.y);
            }
            textureCoordinatesPointer.flip();
        }

        return textureCoordinatesPointer;
    }

    private IntBuffer indicesPointer;

    // Return a pointer to the vertex indicies data
    public IntBuffer getIndicesPointer() {
        if (indicesPointer == null) {
            indicesPointer = BufferUtils.createIntBuffer(mIndices.size());
            for (Integer i : mIndices) {
                indicesPointer.put(i);
            }
            indicesPointer.flip();
        }

        return indicesPointer;
    }

    // Return a reference to a texture of the mesh
    public Texture2D getTexture(int part) {
        return mTextures.get(part);
    }

    // Set a texture to a part of the mesh
    public void setTexture(Texture2D texture, int part) {
        mTextures.put(part, texture);
    }

    // Constructor
    public Mesh() {
    }

    // Destroy the mesh
    public void destroy() {

        mVertices.clear();
        mNormals.clear();
        mTangents.clear();
        mIndices.clear();
        mColors.clear();
        mUVs.clear();
        mTextures.clear();
    }

    // Compute the normals of the mesh
    public void calculateNormals() {

        mNormals = new ArrayList<>();
        for (int i = 0; i < getNbVertices(); i++) {
            mNormals.add(i, new Vector3());
        }

        // For each triangular face
        for (int i = 0; i < getNbFaces(); i++) {

            // Get the three vertices index of the current face
            int i1 = getVertexIndexInFace(i, 0);
            int i2 = getVertexIndexInFace(i, 1);
            int i3 = getVertexIndexInFace(i, 2);

            assert (i1 < getNbVertices());
            assert (i2 < getNbVertices());
            assert (i3 < getNbVertices());

            // Compute the normal of the face
            Vector3 p = getVertex(i1);
            Vector3 q = getVertex(i2);
            Vector3 r = getVertex(i3);
            Vector3 normal = (q.operatorSubtract(p)).cross(r.operatorSubtract(p)).normalize();

            // Add the face surface normal to the sum of normals at
            // each vertex of the face
            mNormals.get(i1).operatorAddEqual(normal);
            mNormals.get(i2).operatorAddEqual(normal);
            mNormals.get(i3).operatorAddEqual(normal);
        }

        // Normalize the normal at each vertex
        for (int i = 0; i < getNbVertices(); i++) {
            mNormals.get(i).normalize();
        }
    }

    // Compute the tangents of the mesh
    public void calculateTangents() {

        mTangents = new ArrayList<>(getNbVertices());

        // For each face
        for (int i = 0; i < getNbFaces(); i++) {

            // Get the three vertices index of the face
            int v1 = getVertexIndexInFace(i, 0);
            int v2 = getVertexIndexInFace(i, 1);
            int v3 = getVertexIndexInFace(i, 2);

            assert (v1 < getNbVertices());
            assert (v2 < getNbVertices());
            assert (v3 < getNbVertices());

            // Get the vertices positions
            Vector3 p = getVertex(v1);
            Vector3 q = getVertex(v2);
            Vector3 r = getVertex(v3);

            // Get the texture coordinates of each vertex
            Vector2 uvP = getUV(v1);
            Vector2 uvQ = getUV(v2);
            Vector2 uvR = getUV(v3);

            // Get the three edges
            Vector3 edge1 = q.operatorSubtract(p);
            Vector3 edge2 = r.operatorSubtract(p);
            Vector2 edge1UV = uvQ.operatorSubtract(uvP);
            Vector2 edge2UV = uvR.operatorSubtract(uvP);

            float cp = edge1UV.y * edge2UV.x - edge1UV.x * edge2UV.y;

            // Compute the tangent
            if (cp != 0.0f) {
                float factor = 1.0f / cp;
                Vector3 tangent = new Vector3(edge1.operatorMultiplyEqual(-edge2UV.y).operatorAddEqual(edge2.operatorMultiplyEqual(edge1UV.y))).operatorMultiplyEqual(factor);
                tangent.normalize();
                mTangents.add(v1, tangent);
                mTangents.add(v2, tangent);
                mTangents.add(v3, tangent);
            }
        }
    }

    // Calculate the bounding box of the mesh
    public void calculateBoundingBox(Vector3 min, Vector3 max) {

        // If the mesh contains vertices
        if (!mVertices.isEmpty()) {

            min.operatorEqual(mVertices.get(0));
            max.operatorEqual(mVertices.get(0));

            // For each vertex of the mesh
            for (Vector3 it : mVertices) {

                if (it.x < min.x) {
                    min.x = it.x;
                } else if (it.x > max.x) {
                    max.x = it.x;
                }

                if (it.y < min.y) {
                    min.y = it.y;
                } else if (it.y > max.y) {
                    max.y = it.y;
                }

                if (it.z < min.z) {
                    min.z = it.z;
                } else if (it.z > max.z) {
                    max.z = it.z;
                }
            }
        } else {
            System.out.println("Error : Impossible to calculate the bounding box of the mesh because there are no vertices !");
            assert (false);
        }
    }

    // Scale of vertices of the mesh using a given factor
    public void scaleVertices(float factor) {

        // For each vertex
        for (int i = 0; i < getNbVertices(); i++) {
            mVertices.get(i).operatorMultiplyEqual(factor);
        }
    }

}
