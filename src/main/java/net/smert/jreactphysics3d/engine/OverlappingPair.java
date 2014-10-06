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
package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a pair of two bodies that are overlapping during the broad-phase collision detection. It is
 * created when the two bodies start to overlap and is destroyed when they do not overlap anymore. This class contains a
 * contact manifold that store all the contact points between the two bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class OverlappingPair {

    // Pointer to the first body of the contact
    private final CollisionBody body1;

    // Pointer to the second body of the contact
    private final CollisionBody body2;

    // Persistent contact manifold
    private final ContactManifold contactManifold;

    // Cached previous separating axis
    private final Vector3 cachedSeparatingAxis;

    // Constructor
    public OverlappingPair(CollisionBody body1, CollisionBody body2) {
        this.body1 = body1;
        this.body2 = body2;
        contactManifold = new ContactManifold(body1, body2);
        cachedSeparatingAxis = new Vector3(1.0f, 1.0f, 1.0f);
    }

    // Add a contact to the contact manifold
    public void addContact(ContactPoint contact) {
        contactManifold.addContactPoint(contact);
    }

    // Return the pointer to first body
    public CollisionBody getBody1() {
        return body1;
    }

    // Return the pointer to second body
    public CollisionBody getBody2() {
        return body2;
    }

    // Return the contact manifold
    public ContactManifold getContactManifold() {
        return contactManifold;
    }

    // Return the number of contact points in the contact manifold
    public int getNumContactPoints() {
        return contactManifold.getNumContactPoints();
    }

    // Return the cached separating axis
    public Vector3 getCachedSeparatingAxis() {
        return cachedSeparatingAxis;
    }

    // Set the cached separating axis
    public void setCachedSeparatingAxis(Vector3 axis) {
        cachedSeparatingAxis.set(axis);
    }

    // Update the contact manifold
    public void update() {
        contactManifold.update(body1.getTransform(), body2.getTransform());
    }

}
