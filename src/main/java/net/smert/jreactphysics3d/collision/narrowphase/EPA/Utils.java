package net.smert.jreactphysics3d.collision.narrowphase.EPA;

/**
 * Methods shared by multiple classes.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Utils {

    // Link an edge with another one. It means that the current edge of a triangle will
    // be associated with the edge of another triangle in order that both triangles
    // are neighbour along both edges).
    public static boolean link(EdgeEPA edge0, EdgeEPA edge1) {
        boolean isPossible = (edge0.getSourceVertexIndex() == edge1.getTargetVertexIndex()
                && edge0.getTargetVertexIndex() == edge1.getSourceVertexIndex());

        if (isPossible) {
            edge0.getOwnerTriangle().mAdjacentEdges[edge0.getIndex()] = edge1;
            edge1.getOwnerTriangle().mAdjacentEdges[edge1.getIndex()] = edge0;
        }

        return isPossible;
    }

    // Make an half link of an edge with another one from another triangle. An half-link
    // between an edge "edge0" and an edge "edge1" represents the fact that "edge1" is an
    // adjacent edge of "edge0" but not the opposite. The opposite edge connection will
    // be made later.
    public static void halfLink(EdgeEPA edge0, EdgeEPA edge1) {
        assert (edge0.getSourceVertexIndex() == edge1.getTargetVertexIndex()
                && edge0.getTargetVertexIndex() == edge1.getSourceVertexIndex());

        // Link
        edge0.getOwnerTriangle().mAdjacentEdges[edge0.getIndex()] = edge1;
    }

}
