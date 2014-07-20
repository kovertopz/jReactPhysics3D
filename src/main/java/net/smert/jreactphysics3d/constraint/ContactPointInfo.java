package net.smert.jreactphysics3d.constraint;

import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This structure contains informations about a collision contact computed during the narrow-phase collision detection.
 * Those informations are used to compute the contact set for a contact between two bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactPointInfo {

    /// First rigid body of the constraint
    public RigidBody body1;

    /// Second rigid body of the constraint
    public RigidBody body2;

    /// Normal vector the the collision contact in world space
    public Vector3 normal;

    /// Penetration depth of the contact
    public float penetrationDepth;

    /// Contact point of body 1 in local space of body 1
    public Vector3 localPoint1;

    /// Contact point of body 2 in local space of body 2
    public Vector3 localPoint2;

    /// Private copy-constructor
    private ContactPointInfo(ContactPointInfo contactInfo) {
    }

    /// Private assignment operator
    private ContactPointInfo operatorEqual(ContactPointInfo contactInfo) {
        return this;
    }

    /// Constructor
    public ContactPointInfo(Vector3 normal, float penetrationDepth, Vector3 localPoint1, Vector3 localPoint2) {
        this.normal = normal;
        this.penetrationDepth = penetrationDepth;
        this.localPoint1 = localPoint1;
        this.localPoint2 = localPoint2;
    }

}
