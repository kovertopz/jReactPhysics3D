package net.smert.jreactphysics3d.collision.narrowphase.EPA;

import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a triangle face of the current polytope in the EPA algorithm.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TriangleEPA {

    // Indices of the vertices y_i of the triangle
    private final int[] mIndicesVertices = new int[3];

    // Three adjacent edges of the triangle (edges of other triangles)
    final EdgeEPA[] mAdjacentEdges = new EdgeEPA[3];

    // True if the triangle face is visible from the new support point
    private boolean mIsObsolete;

    // Determinant
    private float mDet;

    // Point v closest to the origin on the affine hull of the triangle
    private Vector3 mClosestPoint;

    // Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
    private float mLambda1;

    // Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
    private float mLambda2;

    // Square distance of the point closest point v to the origin
    private float mDistSquare;

    // Constructor
    public TriangleEPA() {
        this(0, 0, 0);
    }

    // Constructor
    public TriangleEPA(int indexVertex1, int indexVertex2, int indexVertex3) {
        mIsObsolete = false;
        mIndicesVertices[0] = indexVertex1;
        mIndicesVertices[1] = indexVertex2;
        mIndicesVertices[2] = indexVertex3;
    }

    // Return an edge of the triangle
    public EdgeEPA getAdjacentEdge(int index) {
        assert (index >= 0 && index < 3);
        return mAdjacentEdges[index];
    }

    // Set an adjacent edge of the triangle
    public void setAdjacentEdge(int index, EdgeEPA edge) {
        assert (index >= 0 && index < 3);
        mAdjacentEdges[index] = edge;
    }

    // Return the square distance  of the closest point to origin
    public float getDistSquare() {
        return mDistSquare;
    }

    // Set the isObsolete value
    public void setIsObsolete(boolean isObsolete) {
        mIsObsolete = isObsolete;
    }

    // Return true if the triangle face is obsolete
    public boolean getIsObsolete() {
        return mIsObsolete;
    }

    // Return the point closest to the origin
    public Vector3 getClosestPoint() {
        return mClosestPoint;
    }

    // Return true if the closest point on affine hull is inside the triangle
    public boolean isClosestPointInternalToTriangle() {
        return (mLambda1 >= 0.0f && mLambda2 >= 0.0f && (mLambda1 + mLambda2) <= mDet);
    }

    // Return true if the triangle is visible from a given vertex
    public boolean isVisibleFromVertex(Vector3[] vertices, int index) {
        Vector3 closestToVert = new Vector3(vertices[index]).subtract(mClosestPoint);
        return (mClosestPoint.dot(closestToVert) > 0.0f);
    }

    // Compute the point of an object closest to the origin
    public Vector3 computeClosestPointOfObject(Vector3[] supportPointsOfObject) {
        Vector3 p0 = supportPointsOfObject[mIndicesVertices[0]];
        return new Vector3(p0).add(Vector3.operatorMultiply(1.0f / mDet,
                new Vector3(
                        Vector3.operatorMultiply(mLambda1, new Vector3(supportPointsOfObject[mIndicesVertices[1]]).subtract(p0))).add(
                        Vector3.operatorMultiply(mLambda2, new Vector3(supportPointsOfObject[mIndicesVertices[2]]).subtract(p0)))));
    }

    // Access operator
    public int operatorSquareBrackets(int i) {
        assert (i >= 0 && i < 3);
        return mIndicesVertices[i];
    }

    // Compute the point v closest to the origin of this triangle
    public boolean computeClosestPoint(Vector3[] vertices) {
        Vector3 p0 = vertices[mIndicesVertices[0]];

        Vector3 v1 = new Vector3(vertices[mIndicesVertices[1]]).subtract(p0);
        Vector3 v2 = new Vector3(vertices[mIndicesVertices[2]]).subtract(p0);
        float v1Dotv1 = v1.dot(v1);
        float v1Dotv2 = v1.dot(v2);
        float v2Dotv2 = v2.dot(v2);
        float p0Dotv1 = p0.dot(v1);
        float p0Dotv2 = p0.dot(v2);

        // Compute determinant
        mDet = v1Dotv1 * v2Dotv2 - v1Dotv2 * v1Dotv2;

        // Compute lambda values
        mLambda1 = p0Dotv2 * v1Dotv2 - p0Dotv1 * v2Dotv2;
        mLambda2 = p0Dotv1 * v1Dotv2 - p0Dotv2 * v1Dotv1;

        // If the determinant is positive
        if (mDet > 0.0f) {
            // Compute the closest point v
            mClosestPoint = new Vector3(p0).add(Vector3.operatorMultiply(1.0f / mDet,
                    new Vector3(Vector3.operatorMultiply(mLambda1, v1)).add(Vector3.operatorMultiply(mLambda2, v2))));

            // Compute the square distance of closest point to the origin
            mDistSquare = mClosestPoint.dot(mClosestPoint);

            return true;
        }

        return false;
    }

    // Execute the recursive silhouette algorithm from this triangle face.
    // The parameter "vertices" is an array that contains the vertices of the current polytope and the
    // parameter "indexNewVertex" is the index of the new vertex in this array. The goal of the
    // silhouette algorithm is to add the new vertex in the polytope by keeping it convex. Therefore,
    // the triangle faces that are visible from the new vertex must be removed from the polytope and we
    // need to add triangle faces where each face contains the new vertex and an edge of the silhouette.
    // The silhouette is the connected set of edges that are part of the border between faces that
    // are seen and faces that are not seen from the new vertex. This method starts from the nearest
    // face from the new vertex, computes the silhouette and create the new faces from the new vertex in
    // order that we always have a convex polytope. The faces visible from the new vertex are set
    // obselete and will not be considered as being a candidate face in the future.
    public boolean computeSilhouette(Vector3[] vertices, int indexNewVertex, TrianglesStore triangleStore) {

        int first = triangleStore.getNbTriangles();

        // Mark the current triangle as obsolete because it
        setIsObsolete(true);

        // Execute recursively the silhouette algorithm for the adjacent edges of neighboring
        // triangles of the current triangle
        boolean result = mAdjacentEdges[0].computeSilhouette(vertices, indexNewVertex, triangleStore)
                && mAdjacentEdges[1].computeSilhouette(vertices, indexNewVertex, triangleStore)
                && mAdjacentEdges[2].computeSilhouette(vertices, indexNewVertex, triangleStore);

        if (result) {
            int i, j;

            // For each triangle face that contains the new vertex and an edge of the silhouette
            for (i = first, j = triangleStore.getNbTriangles() - 1;
                    i != triangleStore.getNbTriangles(); j = i++) {
                TriangleEPA triangle = triangleStore.operatorSquareBrackets(i);
                Utils.halfLink(triangle.getAdjacentEdge(1), new EdgeEPA(triangle, 1));

                if (!Utils.link(new EdgeEPA(triangle, 0), new EdgeEPA(triangleStore.operatorSquareBrackets(j), 2))) {
                    return false;
                }
            }

        }

        return result;
    }

}
