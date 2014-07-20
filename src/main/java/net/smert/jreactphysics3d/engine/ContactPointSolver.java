package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * Contact solver internal data structure that to store all the information relative to a contact point
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactPointSolver {

    /// Accumulated normal impulse
    public float penetrationImpulse;

    /// Accumulated impulse in the 1st friction direction
    public float friction1Impulse;

    /// Accumulated impulse in the 2nd friction direction
    public float friction2Impulse;

    /// Accumulated split impulse for penetration correction
    public float penetrationSplitImpulse;

    /// Normal vector of the contact
    public Vector3 normal;

    /// First friction vector in the tangent plane
    public Vector3 frictionVector1;

    /// Second friction vector in the tangent plane
    public Vector3 frictionVector2;

    /// Old first friction vector in the tangent plane
    public Vector3 oldFrictionVector1;

    /// Old second friction vector in the tangent plane
    public Vector3 oldFrictionVector2;

    /// Vector from the body 1 center to the contact point
    public Vector3 r1;

    /// Vector from the body 2 center to the contact point
    public Vector3 r2;

    /// Cross product of r1 with 1st friction vector
    public Vector3 r1CrossT1;

    /// Cross product of r1 with 2nd friction vector
    public Vector3 r1CrossT2;

    /// Cross product of r2 with 1st friction vector
    public Vector3 r2CrossT1;

    /// Cross product of r2 with 2nd friction vector
    public Vector3 r2CrossT2;

    /// Cross product of r1 with the contact normal
    public Vector3 r1CrossN;

    /// Cross product of r2 with the contact normal
    public Vector3 r2CrossN;

    /// Penetration depth
    public float penetrationDepth;

    /// Velocity restitution bias
    public float restitutionBias;

    /// Inverse of the matrix K for the penenetration
    public float inversePenetrationMass;

    /// Inverse of the matrix K for the 1st friction
    public float inverseFriction1Mass;

    /// Inverse of the matrix K for the 2nd friction
    public float inverseFriction2Mass;

    /// True if the contact was existing last time step
    public boolean isRestingContact;

    /// Pointer to the external contact
    public ContactPoint externalContact;

}
