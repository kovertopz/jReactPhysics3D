package net.smert.jreactphysics3d.constraint;

import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a collision contact point between two bodies in the physics engine.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactPoint {

    /// First rigid body of the contact
    private RigidBody mBody1;

    /// Second rigid body of the contact
    private RigidBody mBody2;

    /// Normal vector of the contact (From body1 toward body2) in world space
    private Vector3 mNormal;

    /// Penetration depth
    private float mPenetrationDepth;

    /// Contact point on body 1 in local space of body 1
    private Vector3 mLocalPointOnBody1;

    /// Contact point on body 2 in local space of body 2
    private Vector3 mLocalPointOnBody2;

    /// Contact point on body 1 in world space
    private Vector3 mWorldPointOnBody1;

    /// Contact point on body 2 in world space
    private Vector3 mWorldPointOnBody2;

    /// True if the contact is a resting contact (exists for more than one time step)
    private boolean mIsRestingContact;

    /// Two orthogonal vectors that span the tangential friction plane
    private Vector3[] mFrictionVectors = new Vector3[2];

    /// Cached penetration impulse
    private float mPenetrationImpulse;

    /// Cached first friction impulse
    private float mFrictionImpulse1;

    /// Cached second friction impulse
    private float mFrictionImpulse2;

    /// Private copy-constructor
    private ContactPoint(ContactPoint contact) {
    }

    /// Private assignment operator
    private ContactPoint operatorEqual(ContactPoint contact) {
        return this;
    }

    // Constructor
    public ContactPoint(ContactPointInfo contactInfo) {
        mBody1 = contactInfo.body1;
        mBody2 = contactInfo.body2;
        mNormal = contactInfo.normal;
        mPenetrationDepth = contactInfo.penetrationDepth;
        mLocalPointOnBody1 = contactInfo.localPoint1;
        mLocalPointOnBody2 = contactInfo.localPoint2;
        mWorldPointOnBody1 = contactInfo.body1.getTransform() * contactInfo.localPoint1;
        mWorldPointOnBody2 = contactInfo.body2.getTransform() * contactInfo.localPoint2;
        mIsRestingContact = false;

        mFrictionVectors[0] = new Vector3(0.0f, 0.0f, 0.0f);
        mFrictionVectors[1] = new Vector3(0.0f, 0.0f, 0.0f);

        assert (mPenetrationDepth > 0.0f);
    }

    // Return the reference to the body 1
    public RigidBody getBody1() {
        return mBody1;
    }

    // Return the reference to the body 2
    public RigidBody getBody2() {
        return mBody2;
    }

    // Return the normal vector of the contact
    public Vector3 getNormal() {
        return mNormal;
    }

    // Set the penetration depth of the contact
    public void setPenetrationDepth(float penetrationDepth) {
        this.mPenetrationDepth = penetrationDepth;
    }

    // Return the contact point on body 1
    public Vector3 getLocalPointOnBody1() {
        return mLocalPointOnBody1;
    }

    // Return the contact point on body 2
    public Vector3 getLocalPointOnBody2() {
        return mLocalPointOnBody2;
    }

    // Return the contact world point on body 1
    public Vector3 getWorldPointOnBody1() {
        return mWorldPointOnBody1;
    }

    // Return the contact world point on body 2
    public Vector3 getWorldPointOnBody2() {
        return mWorldPointOnBody2;
    }

    // Return the cached penetration impulse
    public float getPenetrationImpulse() {
        return mPenetrationImpulse;
    }

    // Return the cached first friction impulse
    public float getFrictionImpulse1() {
        return mFrictionImpulse1;
    }

    // Return the cached second friction impulse
    public float getFrictionImpulse2() {
        return mFrictionImpulse2;
    }

    // Set the cached penetration impulse
    public void setPenetrationImpulse(float impulse) {
        mPenetrationImpulse = impulse;
    }

    // Set the first cached friction impulse
    public void setFrictionImpulse1(float impulse) {
        mFrictionImpulse1 = impulse;
    }

    // Set the second cached friction impulse
    public void setFrictionImpulse2(float impulse) {
        mFrictionImpulse2 = impulse;
    }

    // Set the contact world point on body 1
    public void setWorldPointOnBody1(Vector3 worldPoint) {
        mWorldPointOnBody1 = worldPoint;
    }

    // Set the contact world point on body 2
    public void setWorldPointOnBody2(Vector3 worldPoint) {
        mWorldPointOnBody2 = worldPoint;
    }

    // Return true if the contact is a resting contact
    public boolean getIsRestingContact() {
        return mIsRestingContact;
    }

    // Set the mIsRestingContact variable
    public void setIsRestingContact(boolean isRestingContact) {
        mIsRestingContact = isRestingContact;
    }

    // Get the first friction vector
    public Vector3 getFrictionVector1() {
        return mFrictionVectors[0];
    }

    // Set the first friction vector
    public void setFrictionVector1(Vector3 frictionVector1) {
        mFrictionVectors[0] = frictionVector1;
    }

    // Get the second friction vector
    public Vector3 getFrictionVector2() {
        return mFrictionVectors[1];
    }

    // Set the second friction vector
    public void setFrictionVector2(Vector3 frictionVector2) {
        mFrictionVectors[1] = frictionVector2;
    }

    // Return the penetration depth of the contact
    public float getPenetrationDepth() {
        return mPenetrationDepth;
    }

    // Return the number of bytes used by the contact point
    public int getSizeInBytes() {
        return 4;
    }

}
