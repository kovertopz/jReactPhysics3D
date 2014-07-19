package net.smert.jreactphysics3d.collision.narrowphase.EPA;

/**
 * This class allows the comparison of two triangles in the heap The comparison between two triangles is made using
 * their square distance to the closest point to the origin. The goal is that in the heap, the first triangle is the one
 * with the smallest square distance.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TriangleComparison {

    public boolean operatorParathesis(TriangleEPA face1, TriangleEPA face2) {
        return (face1.getDistSquare() > face2.getDistSquare());
    }

}
