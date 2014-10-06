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
package net.smert.jreactphysics3d.collision.narrowphase;

import net.smert.jreactphysics3d.collision.BroadPhasePair;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
import net.smert.jreactphysics3d.mathematics.Transform;

/**
 * This class is an abstract class that represents an algorithm used to perform the narrow-phase of a collision
 * detection. The goal of the narrow phase algorithm is to compute contact informations of a collision between two
 * bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class NarrowPhaseAlgorithm {

    // Overlapping pair of the bodies currently tested for collision
    protected BroadPhasePair currentOverlappingPair;

    // Constructor
    public NarrowPhaseAlgorithm() {
    }

    // Set the current overlapping pair of bodies
    public void setCurrentOverlappingPair(BroadPhasePair overlappingPair) {
        currentOverlappingPair = overlappingPair;
    }

    // Return true and compute a contact info if the two bounding volume collide
    public abstract boolean testCollision(
            CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            ContactPointInfo contactInfo);

}
