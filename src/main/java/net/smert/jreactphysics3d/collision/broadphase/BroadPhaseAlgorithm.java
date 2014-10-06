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

import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.collision.CollisionDetection;
import net.smert.jreactphysics3d.collision.shapes.AABB;

/**
 * This class is an abstract class that represents an algorithm used to perform the broad-phase of a collision
 * detection. The goal of the broad-phase algorithm is to compute the pair of bodies that can collide. But it's
 * important to understand that the broad-phase doesn't compute only body pairs that can collide but could also pairs of
 * body that doesn't collide but are very close. The goal of the broad-phase is to remove pairs of body that cannot
 * collide in order to avoid to much bodies to be tested in the narrow-phase.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class BroadPhaseAlgorithm {

    // Reference to the collision detection object
    protected CollisionDetection collisionDetection;

    // Pair manager containing the overlapping pairs
    protected PairManager pairManager;

    // Constructor
    public BroadPhaseAlgorithm(CollisionDetection collisionDetection) {
        this.collisionDetection = collisionDetection;
        pairManager = new PairManager(collisionDetection);
    }

    // Notify the broad-phase about a new object in the world
    public abstract void addObject(CollisionBody body, AABB aabb);

    // Notify the broad-phase about an object that has been removed from the world
    public abstract void removeObject(CollisionBody body);

    // Notify the broad-phase that the AABB of an object has changed
    public abstract void updateObject(CollisionBody body, AABB aabb);

}
