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
    private CollisionBody mBody1;

    // Pointer to the second body of the contact
    private CollisionBody mBody2;

    // Persistent contact manifold
    private ContactManifold mContactManifold;

    // Cached previous separating axis
    private Vector3 mCachedSeparatingAxis;

    // Constructor
    public OverlappingPair(CollisionBody body1, CollisionBody body2) {
        mBody1 = body1;
        mBody2 = body2;
        mContactManifold = new ContactManifold(body1, body2);
        mCachedSeparatingAxis = new Vector3(1.0f, 1.0f, 1.0f);
    }

    // Return the pointer to first body
    public CollisionBody getBody1() {
        return mBody1;
    }

    // Return the pointer to second body
    public CollisionBody getBody2() {
        return mBody2;
    }

    // Add a contact to the contact manifold
    public void addContact(ContactPoint contact) {
        mContactManifold.addContactPoint(contact);
    }

    // Update the contact manifold
    public void update() {
        mContactManifold.update(mBody1.getTransform(), mBody2.getTransform());
    }

    // Return the cached separating axis
    public Vector3 getCachedSeparatingAxis() {
        return mCachedSeparatingAxis;
    }

    // Set the cached separating axis
    public void setCachedSeparatingAxis(Vector3 axis) {
        mCachedSeparatingAxis = axis;
    }

    // Return the number of contact points in the contact manifold
    public int getNbContactPoints() {
        return mContactManifold.getNbContactPoints();
    }

    // Return the contact manifold
    public ContactManifold getContactManifold() {
        return mContactManifold;
    }

}
