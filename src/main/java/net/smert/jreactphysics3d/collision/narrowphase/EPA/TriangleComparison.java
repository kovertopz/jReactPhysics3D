package net.smert.jreactphysics3d.collision.narrowphase.EPA;

import java.util.Comparator;

/**
 * This class allows the comparison of two triangles in the heap The comparison between two triangles is made using
 * their square distance to the closest point to the origin. The goal is that in the heap, the first triangle is the one
 * with the smallest square distance.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TriangleComparison implements Comparator<TriangleEPA> {

    @Override
    public int compare(TriangleEPA face1, TriangleEPA face2) {
        if (face1.getDistanceSquare() == face2.getDistanceSquare()) {
            return 0;
        }
        return face1.getDistanceSquare() > face2.getDistanceSquare() ? 1 : -1;
    }

}
