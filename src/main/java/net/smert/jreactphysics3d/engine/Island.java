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

import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.constraint.Joint;

/**
 * An island represent an isolated group of awake bodies that are connected with each other by some contraints (contacts
 * or joints).
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Island {

    // Current number of bodies in the island
    private int numBodies;

    // Current number of contact manifold in the island
    private int numContactManifolds;

    // Current number of joints in the island
    private int numJoints;

    // Array with all the contact manifolds between bodies of the island
    private final ContactManifold[] contactManifolds;

    // Array with all the joints between bodies of the island
    private final Joint[] joints;

    // Array with all the bodies of the island
    private final RigidBody[] bodies;

    // Constructor
    public Island(int numMaxBodies, int numMaxContactManifolds, int numMaxJoints) {
        numBodies = 0;
        numContactManifolds = 0;
        numJoints = 0;
        bodies = new RigidBody[numMaxBodies];
        contactManifolds = new ContactManifold[numMaxContactManifolds];
        joints = new Joint[numMaxJoints];
    }

    // Add a body into the island
    public void addBody(RigidBody body) {
        assert (!body.isSleeping());
        bodies[numBodies] = body;
        numBodies++;
    }

    // Add a contact manifold into the island
    public void addContactManifold(ContactManifold contactManifold) {
        contactManifolds[numContactManifolds] = contactManifold;
        numContactManifolds++;
    }

    // Add a joint into the island
    public void addJoint(Joint joint) {
        joints[numJoints] = joint;
        numJoints++;
    }

    // Return the number of bodies in the island
    public int getNumBodies() {
        return numBodies;
    }

    // Return the number of contact manifolds in the island
    public int getNumContactManifolds() {
        return numContactManifolds;
    }

    // Return the number of joints in the island
    public int getNumJoints() {
        return numJoints;
    }

    // Return a pointer to the array of contact manifolds
    public ContactManifold[] getContactManifolds() {
        return contactManifolds;
    }

    // Return a pointer to the array of joints
    public Joint[] getJoints() {
        return joints;
    }

    // Return a pointer to the array of bodies
    public RigidBody[] getBodies() {
        return bodies;
    }

}
