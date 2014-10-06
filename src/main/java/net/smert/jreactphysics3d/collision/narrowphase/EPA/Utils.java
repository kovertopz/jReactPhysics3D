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

/**
 * Methods shared by multiple classes.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Utils {

    // Make an half link of an edge with another one from another triangle. An half-link
    // between an edge "edge0" and an edge "edge1" represents the fact that "edge1" is an
    // adjacent edge of "edge0" but not the opposite. The opposite edge connection will
    // be made later.
    public static void halfLink(EdgeEPA edge0, EdgeEPA edge1) {
        assert (edge0.getSourceVertexIndex() == edge1.getTargetVertexIndex()
                && edge0.getTargetVertexIndex() == edge1.getSourceVertexIndex());

        // Link
        edge0.getOwnerTriangle().setAdjacentEdge(edge0.getIndex(), edge1);
    }

    // Link an edge with another one. It means that the current edge of a triangle will
    // be associated with the edge of another triangle in order that both triangles
    // are neighbour along both edges).
    public static boolean link(EdgeEPA edge0, EdgeEPA edge1) {
        boolean isPossible = (edge0.getSourceVertexIndex() == edge1.getTargetVertexIndex()
                && edge0.getTargetVertexIndex() == edge1.getSourceVertexIndex());

        if (isPossible) {
            edge0.getOwnerTriangle().setAdjacentEdge(edge0.getIndex(), edge1);
            edge1.getOwnerTriangle().setAdjacentEdge(edge1.getIndex(), edge0);
        }

        return isPossible;
    }

}
