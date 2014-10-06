/*
 * ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/
 * Copyright (c) 2010-2013 Daniel Chappuis
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim
 *    that you wrote the original software. If you use this software in a
 *    product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * This file has been modified during the port to Java and differ from the source versions.
 */
package net.smert.jreactphysics3d.collision.narrowphase.EPA;

import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents an edge of the current polytope in the EPA algorithm.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class EdgeEPA {

    // Index of the edge in the triangle (between 0 and 2).
    // The edge with index i connect triangle vertices i and (i+1 % 3)
    private final int index;

    // Pointer to the triangle that contains this edge
    private final TriangleEPA ownerTriangle;

    // Constructor
    public EdgeEPA(TriangleEPA ownerTriangle, int index) {
        assert (index >= 0 && index < 3);
        this.index = index;
        this.ownerTriangle = ownerTriangle;
    }

    // Execute the recursive silhouette algorithm from this edge
    public boolean computeSilhouette(Vector3[] vertices, int indexNewVertex, TrianglesStore triangleStore) {

        // If the edge has not already been visited
        if (!ownerTriangle.getIsObsolete()) {

            // If the triangle of this edge is not visible from the given point
            if (!ownerTriangle.isVisibleFromVertex(vertices, indexNewVertex)) {

                TriangleEPA triangle = triangleStore
                        .newTriangle(vertices, indexNewVertex, getTargetVertexIndex(), getSourceVertexIndex());

                // If the triangle has been created
                if (triangle != null) {
                    Utils.halfLink(new EdgeEPA(triangle, 1), this);
                    return true;
                }

                return false;
            } else {

                // The current triangle is visible and therefore obsolete
                ownerTriangle.setIsObsolete(true);

                int backup = triangleStore.getNumTriangles();

                if (!ownerTriangle.getAdjacentEdge(indexOfNextCounterClockwiseEdge(index))
                        .computeSilhouette(vertices, indexNewVertex, triangleStore)) {

                    ownerTriangle.setIsObsolete(false);

                    TriangleEPA triangle = triangleStore
                            .newTriangle(vertices, indexNewVertex, getTargetVertexIndex(), getSourceVertexIndex());

                    // If the triangle has been created
                    if (triangle != null) {
                        Utils.halfLink(new EdgeEPA(triangle, 1), this);
                        return true;
                    }

                    return false;
                } else if (!ownerTriangle.getAdjacentEdge(indexOfPreviousCounterClockwiseEdge(index))
                        .computeSilhouette(vertices, indexNewVertex, triangleStore)) {

                    ownerTriangle.setIsObsolete(false);

                    triangleStore.setNumTriangles(backup);

                    TriangleEPA triangle = triangleStore
                            .newTriangle(vertices, indexNewVertex, getTargetVertexIndex(), getSourceVertexIndex());

                    if (triangle != null) {
                        Utils.halfLink(new EdgeEPA(triangle, 1), this);
                        return true;
                    }

                    return false;
                }
            }
        }

        return true;
    }

    // Return the edge index
    public int getIndex() {
        return index;
    }

    // Return the index of the source vertex of the edge (vertex starting the edge)
    public int getSourceVertexIndex() {
        return ownerTriangle.getIndexVertex(index);
    }

    // Return the index of the target vertex of the edge (vertex ending the edge)
    public int getTargetVertexIndex() {
        return ownerTriangle.getIndexVertex(indexOfNextCounterClockwiseEdge(index));
    }

    // Return the pointer to the owner triangle
    public TriangleEPA getOwnerTriangle() {
        return ownerTriangle;
    }

    // Return the index of the next counter-clockwise edge of the ownver triangle
    public int indexOfNextCounterClockwiseEdge(int index) {
        return (index + 1) % 3;
    }

    // Return the index of the previous counter-clockwise edge of the ownver triangle
    public int indexOfPreviousCounterClockwiseEdge(int index) {
        return (index + 2) % 3;
    }

}
