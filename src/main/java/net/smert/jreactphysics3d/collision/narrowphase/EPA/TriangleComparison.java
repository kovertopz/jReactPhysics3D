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
