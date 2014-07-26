package net.smert.jreactphysics3d.collision.narrowphase.EPA;

import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class stores several triangles of the polytope in the EPA algorithm.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TrianglesStore {

    private static final int MAX_TRIANGLES = 200;     // Maximum number of triangles

    // Triangles
    private TriangleEPA[] mTriangles = new TriangleEPA[MAX_TRIANGLES];

    // Number of triangles
    private int mNbTriangles;

    // Constructor
    public TrianglesStore() {
        mNbTriangles = 0;
    }

    // Clear all the storage
    public void clear() {
        mNbTriangles = 0;
    }

    // Return the number of triangles
    public int getNbTriangles() {
        return mNbTriangles;
    }

    public void setNbTriangles(int backup) {
        mNbTriangles = backup;
    }

    // Return the last triangle
    public TriangleEPA last() {
        assert (mNbTriangles > 0);
        return mTriangles[mNbTriangles - 1];
    }

    // Create a new triangle
    public TriangleEPA newTriangle(Vector3[] vertices, int v0, int v1, int v2) {
        TriangleEPA newTriangle = null;

        // If we have not reached the maximum number of triangles
        if (mNbTriangles != MAX_TRIANGLES) {
            newTriangle = new TriangleEPA(v0, v1, v2);
            mTriangles[mNbTriangles] = newTriangle;
            mNbTriangles++;
            if (!newTriangle.computeClosestPoint(vertices)) {
                mNbTriangles--;
                newTriangle = null;
                mTriangles[mNbTriangles] = null;
            }
        }

        // Return the new triangle
        return newTriangle;
    }

    // Access operator
    public TriangleEPA operatorSquareBrackets(int i) {
        return mTriangles[i];
    }

}
