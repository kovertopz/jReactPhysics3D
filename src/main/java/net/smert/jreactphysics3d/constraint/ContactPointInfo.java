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

    // Penetration depth of the contact
    private float penetrationDepth;

    // First rigid body of the constraint
    private RigidBody body1;

    // Second rigid body of the constraint
    private RigidBody body2;

    // Normal vector the the collision contact in world space
    private final Vector3 normal;

    // Contact point of body 1 in local space of body 1
    private final Vector3 localPoint1;

    // Contact point of body 2 in local space of body 2
    private final Vector3 localPoint2;

    // Constructor
    public ContactPointInfo() {
        this.normal = new Vector3();
        this.penetrationDepth = 0.0f;
        this.localPoint1 = new Vector3();
        this.localPoint2 = new Vector3();
    }

    public float getPenetrationDepth() {
        return penetrationDepth;
    }

    public RigidBody getBody1() {
        return body1;
    }

    public void setBody1(RigidBody body1) {
        this.body1 = body1;
    }

    public RigidBody getBody2() {
        return body2;
    }

    public void setBody2(RigidBody body2) {
        this.body2 = body2;
    }

    public Vector3 getNormal() {
        return normal;
    }

    public Vector3 getLocalPoint1() {
        return localPoint1;
    }

    public Vector3 getLocalPoint2() {
        return localPoint2;
    }

    public void setCollisionData(Vector3 normal, float penetrationDepth, Vector3 localPoint1, Vector3 localPoint2) {
        this.normal.set(normal);
        this.penetrationDepth = penetrationDepth;
        this.localPoint1.set(localPoint1);
        this.localPoint2.set(localPoint2);
    }

}
