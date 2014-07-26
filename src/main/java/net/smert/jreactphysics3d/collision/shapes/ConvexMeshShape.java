package net.smert.jreactphysics3d.collision.shapes;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a convex mesh shape. In order to create a convex mesh shape, you need to indicate the
 * local-space position of the mesh vertices. You do it either by passing a vertices array to the constructor or using
 * the addVertex() method. Make sure that the set of vertices that you use to create the shape are indeed part of a
 * convex mesh. The center of mass of the shape will be at the origin of the local-space geometry that you use to create
 * the mesh. The method used for collision detection with a convex mesh shape has an O(n) running time with "n" beeing
 * the number of vertices in the mesh. Therefore, you should try not to use too many vertices. However, it is possible
 * to speed up the collision detection by using the edges information of your mesh. The running time of the collision
 * detection that uses the edges is almost O(1) constant time at the cost of additional memory used to store the
 * vertices. You can indicate edges information with the addEdge() method. Then, you must use the
 * setIsEdgesInformationUsed(true) method in order to use the edges information for collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ConvexMeshShape extends CollisionShape {

    // Array with the vertices of the mesh
    private List<Vector3> mVertices;

    // Number of vertices in the mesh
    private int mNbVertices;

    // Mesh minimum bounds in the three local x, y and z directions
    private Vector3 mMinBounds;

    // Mesh maximum bounds in the three local x, y and z directions
    private Vector3 mMaxBounds;

    // True if the shape contains the edges of the convex mesh in order to
    // make the collision detection faster
    private boolean mIsEdgesInformationUsed;

    // Adjacency list representing the edges of the mesh
    private Map<Integer, List<Integer>> mEdgesAdjacencyList;

    // Cached support vertex index (previous support vertex)
    private int mCachedSupportVertex;

    // Private copy-constructor
    private ConvexMeshShape(ConvexMeshShape shape) {
        super(shape);

        // TODO: Do real copying
        mVertices = shape.mVertices;
        mNbVertices = shape.mNbVertices;
        mMinBounds = shape.mMinBounds;
        mMaxBounds = shape.mMaxBounds;
        mIsEdgesInformationUsed = shape.mIsEdgesInformationUsed;
        mEdgesAdjacencyList = shape.mEdgesAdjacencyList;
        mCachedSupportVertex = shape.mCachedSupportVertex;

        assert (mNbVertices == mVertices.size());
    }

    // Recompute the bounds of the mesh
    private void recalculateBounds() {

        mMinBounds.setToZero();
        mMaxBounds.setToZero();

        // For each vertex of the mesh
        for (int i = 0; i < mNbVertices; i++) {

            if (mVertices.get(i).x > mMaxBounds.x) {
                mMaxBounds.x = mVertices.get(i).x;
            }
            if (mVertices.get(i).x < mMinBounds.x) {
                mMinBounds.x = mVertices.get(i).x;
            }

            if (mVertices.get(i).y > mMaxBounds.y) {
                mMaxBounds.y = mVertices.get(i).y;
            }
            if (mVertices.get(i).y < mMinBounds.y) {
                mMinBounds.y = mVertices.get(i).y;
            }

            if (mVertices.get(i).z > mMaxBounds.z) {
                mMaxBounds.z = mVertices.get(i).z;
            }
            if (mVertices.get(i).z < mMinBounds.z) {
                mMinBounds.z = mVertices.get(i).z;
            }
        }

        // Add the object margin to the bounds
        mMaxBounds.operatorAddEqual(new Vector3(mMargin, mMargin, mMargin));
        mMinBounds.operatorSubtractEqual(new Vector3(mMargin, mMargin, mMargin));
    }

    // Constructor to initialize with a array of 3D vertices.
    // This method creates an internal copy of the input vertices.
    public ConvexMeshShape(float[] arrayVertices, int nbVertices, int stride, float margin) {
        super(CollisionShapeType.CONVEX_MESH, margin);

        assert (arrayVertices != null);
        assert (nbVertices > 0);
        assert (stride > 0);

        mVertices = new ArrayList<>();
        mNbVertices = nbVertices;
        mMinBounds = new Vector3();
        mMaxBounds = new Vector3();
        mIsEdgesInformationUsed = false;
        mEdgesAdjacencyList = new HashMap<>();
        mCachedSupportVertex = 0;

        int vertexPointer = 0;

        // Copy all the vertices into the internal array
        for (int i = 0; i < mNbVertices; i++) {
            mVertices.add(new Vector3(arrayVertices[vertexPointer + 0], arrayVertices[vertexPointer + 1], arrayVertices[vertexPointer + 2]));
            vertexPointer += stride;
        }

        // Recalculate the bounds of the mesh
        recalculateBounds();
    }

    // Constructor.
    // If you use this constructor, you will need to set the vertices manually one by one using
    // the addVertex() method.
    public ConvexMeshShape(float margin) {
        super(CollisionShapeType.CONVEX_MESH, margin);

        mVertices = new ArrayList<>();
        mNbVertices = 0;
        mMinBounds = new Vector3();
        mMaxBounds = new Vector3();
        mIsEdgesInformationUsed = false;
        mEdgesAdjacencyList = new HashMap<>();
        mCachedSupportVertex = 0;
    }

    // Add an edge into the convex mesh by specifying the two vertex indices of the edge.
    // Note that the vertex indices start at zero and need to correspond to the order of
    // the vertices in the vertices array in the constructor or the order of the calls
    // of the addVertex() methods that you use to add vertices into the convex mesh.
    public void addEdge(int v1, int v2) {

        assert (v1 >= 0);
        assert (v2 >= 0);

        // If the entry for vertex v1 does not exist in the adjacency list
        if (mEdgesAdjacencyList.containsKey(v1) == false) {
            mEdgesAdjacencyList.put(v1, new ArrayList<Integer>());
        }

        // If the entry for vertex v2 does not exist in the adjacency list
        if (mEdgesAdjacencyList.containsKey(v2) == false) {
            mEdgesAdjacencyList.put(v2, new ArrayList<Integer>());
        }

        // Add the edge in the adjacency list
        mEdgesAdjacencyList.get(v1).add(v2);
        mEdgesAdjacencyList.get(v2).add(v1);
    }

    // Add a vertex into the convex mesh
    public void addVertex(Vector3 vertex) {

        // Add the vertex in to vertices array
        mVertices.add(vertex);
        mNbVertices++;

        // Update the bounds of the mesh
        if (vertex.x > mMaxBounds.x) {
            mMaxBounds.x = vertex.x;
        }
        if (vertex.x < mMinBounds.x) {
            mMinBounds.x = vertex.x;
        }
        if (vertex.y > mMaxBounds.y) {
            mMaxBounds.y = vertex.y;
        }
        if (vertex.y < mMinBounds.y) {
            mMinBounds.y = vertex.y;
        }
        if (vertex.z > mMaxBounds.z) {
            mMaxBounds.z = vertex.z;
        }
        if (vertex.z < mMinBounds.z) {
            mMinBounds.z = vertex.z;
        }
    }

    // Return true if the edges information is used to speed up the collision detection
    public boolean isEdgesInformationUsed() {
        return mIsEdgesInformationUsed;
    }

    // Set the variable to know if the edges information is used to speed up the
    // collision detection
    public void setIsEdgesInformationUsed(boolean isEdgesUsed) {
        mIsEdgesInformationUsed = isEdgesUsed;
    }

    @Override
    public CollisionShape clone() {
        return new ConvexMeshShape(this);
    }

    // Return the local inertia tensor of the collision shape.
    // The local inertia tensor of the convex mesh is approximated using the inertia tensor
    // of its bounding box.
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        float factor = (1.0f / 3.0f) * mass;
        Vector3 realExtent = Vector3.operatorSubtract(mMaxBounds, mMinBounds).operatorMultiplyEqual(0.5f);
        assert (realExtent.x > 0 && realExtent.y > 0 && realExtent.z > 0);
        float xSquare = realExtent.x * realExtent.x;
        float ySquare = realExtent.y * realExtent.y;
        float zSquare = realExtent.z * realExtent.z;
        tensor.setAllValues(factor * (ySquare + zSquare), 0.0f, 0.0f,
                0.0f, factor * (xSquare + zSquare), 0.0f,
                0.0f, 0.0f, factor * (xSquare + ySquare));
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {

        // Get the support point without the margin
        Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);

        // Get the unit direction vector
        Vector3 unitDirection = direction;
        if (direction.lengthSquare() < Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {
            unitDirection.setAllValues(1.0f, 1.0f, 1.0f);
        }
        unitDirection.normalize();

        // Add the margin to the support point and return it
        return Vector3.operatorAdd(supportPoint, unitDirection).operatorMultiplyEqual(mMargin);
    }

    // Return a local support point in a given direction without the object margin.
    // If the edges information is not used for collision detection, this method will go through
    // the whole vertices list and pick up the vertex with the largest dot product in the support
    // direction. This is an O(n) process with "n" being the number of vertices in the mesh.
    // However, if the edges information is used, we can cache the previous support vertex and use
    // it as a start in a hill-climbing (local search) process to find the new support vertex which
    // will be in most of the cases very close to the previous one. Using hill-climbing, this method
    // runs in almost constant time.
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {

        assert (mNbVertices == mVertices.size());

        // If the edges information is used to speed up the collision detection
        if (mIsEdgesInformationUsed) {

            assert (mEdgesAdjacencyList.size() == mNbVertices);

            int maxVertex = mCachedSupportVertex;
            float maxDotProduct = direction.dot(mVertices.get(maxVertex));
            boolean isOptimal;

            // Perform hill-climbing (local search)
            do {
                isOptimal = true;

                assert (mEdgesAdjacencyList.get(maxVertex).size() > 0);

                // For all neighbors of the current vertex
                Iterator it = mEdgesAdjacencyList.get(maxVertex).iterator();
                while (it.hasNext()) {
                    int i = (int) it.next();
                    // Compute the dot product
                    float dotProduct = direction.dot(mVertices.get(i));

                    // If the current vertex is a better vertex (larger dot product)
                    if (dotProduct > maxDotProduct) {
                        maxVertex = i;
                        maxDotProduct = dotProduct;
                        isOptimal = false;
                    }
                }

            } while (!isOptimal);

            // Cache the support vertex
            mCachedSupportVertex = maxVertex;

            // Return the support vertex
            return mVertices.get(maxVertex);
        } else {  // If the edges information is not used

            float maxDotProduct = Defaults.DECIMAL_SMALLEST;
            int indexMaxDotProduct = 0;

            // For each vertex of the mesh
            for (int i = 0; i < mNbVertices; i++) {

                // Compute the dot product of the current vertex
                float dotProduct = direction.dot(mVertices.get(i));

                // If the current dot product is larger than the maximum one
                if (dotProduct > maxDotProduct) {
                    indexMaxDotProduct = i;
                    maxDotProduct = dotProduct;
                }
            }

            assert (maxDotProduct >= 0.0f);

            // Return the vertex with the largest dot product in the support direction
            return mVertices.get(indexMaxDotProduct);
        }
    }

    // Return the local bounds of the shape in x, y and z directions
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {
        min.setAllValues(mMinBounds.x, mMinBounds.y, mMinBounds.z);
        max.setAllValues(mMaxBounds.x, mMaxBounds.y, mMaxBounds.z);
    }

    // Test equality between two cone shapes
    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        ConvexMeshShape otherShape = (ConvexMeshShape) otherCollisionShape;

        assert (mNbVertices == mVertices.size());

        if (mNbVertices != otherShape.mNbVertices) {
            return false;
        }

        // If edges information is used, it means that a collison shape object will store
        // cached data (previous support vertex) and therefore, we should not reuse the shape
        // for another body. Therefore, we consider that all convex mesh shape using edges
        // information are different.
        if (mIsEdgesInformationUsed) {
            return false;
        }

        if (mEdgesAdjacencyList.size() != otherShape.mEdgesAdjacencyList.size()) {
            return false;
        }

        // Check that the vertices are the same
        for (int i = 0; i < mNbVertices; i++) {
            if (mVertices.get(i) != otherShape.mVertices.get(i)) {
                return false;
            }
        }

        // Check that the edges are the same
        for (int i = 0; i < mEdgesAdjacencyList.size(); i++) {

            assert (otherShape.mEdgesAdjacencyList.containsKey(i) == true);
            if (mEdgesAdjacencyList.get(i) != otherShape.mEdgesAdjacencyList.get(i)) {
                return false;
            }
        }

        return true;
    }

}
