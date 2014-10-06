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

import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * Contact solver internal data structure that to store all the information relative to a contact point
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactPointSolver {

    // True if the contact was existing last time step
    public boolean isRestingContact;

    // Accumulated impulse in the 1st friction direction
    public float friction1Impulse;

    // Accumulated impulse in the 2nd friction direction
    public float friction2Impulse;

    // Inverse of the matrix K for the 1st friction
    public float inverseFriction1Mass;

    // Inverse of the matrix K for the 2nd friction
    public float inverseFriction2Mass;

    // Inverse of the matrix K for the penenetration
    public float inversePenetrationMass;

    // Penetration depth
    public float penetrationDepth;

    // Accumulated normal impulse
    public float penetrationImpulse;

    // Accumulated split impulse for penetration correction
    public float penetrationSplitImpulse;

    // Velocity restitution bias
    public float restitutionBias;

    // Pointer to the external contact
    public ContactPoint externalContact;

    // First friction vector in the tangent plane
    public final Vector3 frictionVector1 = new Vector3();

    // Second friction vector in the tangent plane
    public final Vector3 frictionVector2 = new Vector3();

    // Normal vector of the contact
    public final Vector3 normal = new Vector3();

    // Old first friction vector in the tangent plane
    public final Vector3 oldFrictionVector1 = new Vector3();

    // Old second friction vector in the tangent plane
    public final Vector3 oldFrictionVector2 = new Vector3();

    // Vector from the body 1 center to the contact point
    public final Vector3 r1 = new Vector3();

    // Vector from the body 2 center to the contact point
    public final Vector3 r2 = new Vector3();

    // Cross product of r1 with the contact normal
    public final Vector3 r1CrossN = new Vector3();

    // Cross product of r2 with the contact normal
    public final Vector3 r2CrossN = new Vector3();

    // Cross product of r1 with 1st friction vector
    public final Vector3 r1CrossT1 = new Vector3();

    // Cross product of r1 with 2nd friction vector
    public final Vector3 r1CrossT2 = new Vector3();

    // Cross product of r2 with 1st friction vector
    public final Vector3 r2CrossT1 = new Vector3();

    // Cross product of r2 with 2nd friction vector
    public final Vector3 r2CrossT2 = new Vector3();

}
