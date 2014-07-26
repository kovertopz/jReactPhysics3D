package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * Contact solver internal data structure that to store all the information relative to a contact point
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactPointSolver {

    // Accumulated normal impulse
    public float penetrationImpulse;

    // Accumulated impulse in the 1st friction direction
    public float friction1Impulse;

    // Accumulated impulse in the 2nd friction direction
    public float friction2Impulse;

    // Accumulated split impulse for penetration correction
    public float penetrationSplitImpulse;

    // Normal vector of the contact
    public final Vector3 normal = new Vector3();

    // First friction vector in the tangent plane
    public final Vector3 frictionVector1 = new Vector3();

    // Second friction vector in the tangent plane
    public final Vector3 frictionVector2 = new Vector3();

    // Old first friction vector in the tangent plane
    public final Vector3 oldFrictionVector1 = new Vector3();

    // Old second friction vector in the tangent plane
    public final Vector3 oldFrictionVector2 = new Vector3();

    // Vector from the body 1 center to the contact point
    public final Vector3 r1 = new Vector3();

    // Vector from the body 2 center to the contact point
    public final Vector3 r2 = new Vector3();

    // Cross product of r1 with 1st friction vector
    public final Vector3 r1CrossT1 = new Vector3();

    // Cross product of r1 with 2nd friction vector
    public final Vector3 r1CrossT2 = new Vector3();

    // Cross product of r2 with 1st friction vector
    public final Vector3 r2CrossT1 = new Vector3();

    // Cross product of r2 with 2nd friction vector
    public final Vector3 r2CrossT2 = new Vector3();

    // Cross product of r1 with the contact normal
    public final Vector3 r1CrossN = new Vector3();

    // Cross product of r2 with the contact normal
    public final Vector3 r2CrossN = new Vector3();

    // Penetration depth
    public float penetrationDepth;

    // Velocity restitution bias
    public float restitutionBias;

    // Inverse of the matrix K for the penenetration
    public float inversePenetrationMass;

    // Inverse of the matrix K for the 1st friction
    public float inverseFriction1Mass;

    // Inverse of the matrix K for the 2nd friction
    public float inverseFriction2Mass;

    // True if the contact was existing last time step
    public boolean isRestingContact;

    // Pointer to the external contact
    public ContactPoint externalContact;

}
