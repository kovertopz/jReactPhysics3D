package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * Contact solver internal data structure to store all the information relative to a contact manifold.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactManifoldSolver {

    /// Index of body 1 in the constraint solver
    public int indexBody1;

    /// Index of body 2 in the constraint solver
    public int indexBody2;

    /// Inverse of the mass of body 1
    public float massInverseBody1;

    // Inverse of the mass of body 2
    public float massInverseBody2;

    /// Inverse inertia tensor of body 1
    public Matrix3x3 inverseInertiaTensorBody1;

    /// Inverse inertia tensor of body 2
    public Matrix3x3 inverseInertiaTensorBody2;

    /// True if the body 1 is allowed to move
    public boolean isBody1Moving;

    /// True if the body 2 is allowed to move
    public boolean isBody2Moving;

    /// Contact point constraints
    public ContactPointSolver[] contacts = new ContactPointSolver[ContactManifold.MAX_CONTACT_POINTS_IN_MANIFOLD];

    /// Number of contact points
    public int nbContacts;

    /// Mix of the restitution factor for two bodies
    public float restitutionFactor;

    /// Mix friction coefficient for the two bodies
    public float frictionCoefficient;

    /// Pointer to the external contact manifold
    public ContactManifold externalContactManifold;

    // - Variables used when friction constraints are apply at the center of the manifold-//
    /// Average normal vector of the contact manifold
    public Vector3 normal;

    /// Point on body 1 where to apply the friction constraints
    public Vector3 frictionPointBody1;

    /// Point on body 2 where to apply the friction constraints
    public Vector3 frictionPointBody2;

    /// R1 vector for the friction constraints
    public Vector3 r1Friction;

    /// R2 vector for the friction constraints
    public Vector3 r2Friction;

    /// Cross product of r1 with 1st friction vector
    public Vector3 r1CrossT1;

    /// Cross product of r1 with 2nd friction vector
    public Vector3 r1CrossT2;

    /// Cross product of r2 with 1st friction vector
    public Vector3 r2CrossT1;

    /// Cross product of r2 with 2nd friction vector
    public Vector3 r2CrossT2;

    /// Matrix K for the first friction constraint
    public float inverseFriction1Mass;

    /// Matrix K for the second friction constraint
    public float inverseFriction2Mass;

    /// Matrix K for the twist friction constraint
    public float inverseTwistFrictionMass;

    /// First friction direction at contact manifold center
    public Vector3 frictionVector1;

    /// Second friction direction at contact manifold center
    public Vector3 frictionVector2;

    /// Old 1st friction direction at contact manifold center
    public Vector3 oldFrictionVector1;

    /// Old 2nd friction direction at contact manifold center
    public Vector3 oldFrictionVector2;

    /// First friction direction impulse at manifold center
    public float friction1Impulse;

    /// Second friction direction impulse at manifold center
    public float friction2Impulse;

    /// Twist friction impulse at contact manifold center
    public float frictionTwistImpulse;

}
