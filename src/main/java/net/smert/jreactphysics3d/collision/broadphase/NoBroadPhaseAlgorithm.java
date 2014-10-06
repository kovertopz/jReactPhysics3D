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
package net.smert.jreactphysics3d.collision.broadphase;

import java.util.HashSet;
import java.util.Set;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.collision.CollisionDetection;
import net.smert.jreactphysics3d.collision.shapes.AABB;

/**
 * This class implements a broad-phase algorithm that does nothing. It should be use if we don't want to perform a
 * broad-phase for the collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class NoBroadPhaseAlgorithm extends BroadPhaseAlgorithm {

    // All bodies of the world
    protected Set<CollisionBody> bodies;

    // Constructor
    public NoBroadPhaseAlgorithm(CollisionDetection collisionDetection) {
        super(collisionDetection);
        bodies = new HashSet<>();
    }

    // Notify the broad-phase about a new object in the world
    @Override
    public void addObject(CollisionBody body, AABB aabb) {

        // For each body that is already in the world
        for (CollisionBody it : bodies) {

            // Add an overlapping pair with the new body
            pairManager.addPair(it, body);
        }

        // Add the new body into the list of bodies
        bodies.add(body);
    }

    // Notify the broad-phase about an object that has been removed from the world
    @Override
    public void removeObject(CollisionBody body) {

        // For each body that is in the world
        for (CollisionBody it : bodies) {

            if (it.getBodyID() != body.getBodyID()) {

                // Remove the overlapping pair with the new body
                pairManager.removePair(it.getBodyID(), body.getBodyID());
            }
        }

        // Remove the body from the broad-phase
        bodies.remove(body);
    }

    // Notify the broad-phase that the AABB of an object has changed
    @Override
    public void updateObject(CollisionBody body, AABB aabb) {
        // Do nothing
    }

}
