package net.smert.jreactphysics3d.constraint;

import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a collision contact point between two bodies in the physics engine.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactPoint {

    // True if the contact is a resting contact (exists for more than one time step)
    private boolean isRestingContact;

    // Cached first friction impulse
    private float frictionImpulse1;

    // Cached second friction impulse
    private float frictionImpulse2;

    // Penetration depth
    private float penetrationDepth;

    // Cached penetration impulse
    private float penetrationImpulse;

    // First rigid body of the contact
    private final RigidBody body1;

    // Second rigid body of the contact
    private final RigidBody body2;

    // Contact point on body 1 in local space of body 1
    private final Vector3 localPointOnBody1;

    // Contact point on body 2 in local space of body 2
    private final Vector3 localPointOnBody2;

    // Normal vector of the contact (From body1 toward body2) in world space
    private final Vector3 normal;

    // Contact point on body 1 in world space
    private final Vector3 worldPointOnBody1;

    // Contact point on body 2 in world space
    private final Vector3 worldPointOnBody2;

    // Two orthogonal vectors that span the tangential friction plane
    private final Vector3[] frictionVectors = new Vector3[2];

    // Constructor
    public ContactPoint(ContactPointInfo contactInfo) {
        body1 = contactInfo.getBody1();
        body2 = contactInfo.getBody2();
        normal = contactInfo.getNormal();
        penetrationDepth = contactInfo.getPenetrationDepth();
        localPointOnBody1 = contactInfo.getLocalPoint1();
        localPointOnBody2 = contactInfo.getLocalPoint2();
        worldPointOnBody1 = contactInfo.getBody1().getTransform().multiply(localPointOnBody1, new Vector3());
        worldPointOnBody2 = contactInfo.getBody2().getTransform().multiply(localPointOnBody2, new Vector3());
        isRestingContact = false;

        frictionVectors[0] = new Vector3();
        frictionVectors[1] = new Vector3();

        assert (penetrationDepth > 0.0f);
    }

    // Return true if the contact is a resting contact
    public boolean getIsRestingContact() {
        return isRestingContact;
    }

    // Set the mIsRestingContact variable
    public void setIsRestingContact(boolean isRestingContact) {
        this.isRestingContact = isRestingContact;
    }

    // Return the cached first friction impulse
    public float getFrictionImpulse1() {
        return frictionImpulse1;
    }

    // Set the first cached friction impulse
    public void setFrictionImpulse1(float impulse) {
        frictionImpulse1 = impulse;
    }

    // Return the cached second friction impulse
    public float getFrictionImpulse2() {
        return frictionImpulse2;
    }

    // Set the second cached friction impulse
    public void setFrictionImpulse2(float impulse) {
        frictionImpulse2 = impulse;
    }

    // Return the penetration depth of the contact
    public float getPenetrationDepth() {
        return penetrationDepth;
    }

    // Set the penetration depth of the contact
    public void setPenetrationDepth(float penetrationDepth) {
        this.penetrationDepth = penetrationDepth;
    }

    // Return the cached penetration impulse
    public float getPenetrationImpulse() {
        return penetrationImpulse;
    }

    // Set the cached penetration impulse
    public void setPenetrationImpulse(float impulse) {
        penetrationImpulse = impulse;
    }

    // Return the reference to the body 1
    public RigidBody getBody1() {
        return body1;
    }

    // Return the reference to the body 2
    public RigidBody getBody2() {
        return body2;
    }

    // Return the contact point on body 1
    public Vector3 getLocalPointOnBody1() {
        return localPointOnBody1;
    }

    // Return the contact point on body 2
    public Vector3 getLocalPointOnBody2() {
        return localPointOnBody2;
    }

    // Return the normal vector of the contact
    public Vector3 getNormal() {
        return normal;
    }

    // Return the contact world point on body 1
    public Vector3 getWorldPointOnBody1() {
        return worldPointOnBody1;
    }

    // Set the contact world point on body 1
    public void setWorldPointOnBody1(Vector3 worldPoint) {
        worldPointOnBody1.set(worldPoint);
    }

    // Return the contact world point on body 2
    public Vector3 getWorldPointOnBody2() {
        return worldPointOnBody2;
    }

    // Set the contact world point on body 2
    public void setWorldPointOnBody2(Vector3 worldPoint) {
        worldPointOnBody2.set(worldPoint);
    }

    // Get the first friction vector
    public Vector3 getFrictionVector1() {
        return frictionVectors[0];
    }

    // Set the first friction vector
    public void setFrictionVector1(Vector3 frictionVector1) {
        frictionVectors[0] = new Vector3(frictionVector1);
    }

    // Get the second friction vector
    public Vector3 getFrictionVector2() {
        return frictionVectors[1];
    }

    // Set the second friction vector
    public void setFrictionVector2(Vector3 frictionVector2) {
        frictionVectors[1] = new Vector3(frictionVector2);
    }

}
