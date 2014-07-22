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

    /// Array with all the bodies of the island
    private RigidBody[] mBodies;

    /// Array with all the contact manifolds between bodies of the island
    private ContactManifold[] mContactManifolds;

    /// Array with all the joints between bodies of the island
    private Joint[] mJoints;

    /// Current number of bodies in the island
    private int mNbBodies;

    /// Current number of contact manifold in the island
    private int mNbContactManifolds;

    /// Current number of joints in the island
    private int mNbJoints;

    /// Number of bytes allocated for the bodies array
    private int mNbAllocatedBytesBodies;

    /// Number of bytes allocated for the contact manifolds array
    private int mNbAllocatedBytesContactManifolds;

    /// Number of bytes allocated for the joints array
    private int mNbAllocatedBytesJoints;

    // Constructor
    public Island(int nbMaxBodies, int nbMaxContactManifolds, int nbMaxJoints) {
        mBodies = null;
        mContactManifolds = null;
        mJoints = null;
        mNbBodies = 0;
        mNbContactManifolds = 0;
        mNbJoints = 0;

        // Allocate memory for the arrays
        mNbAllocatedBytesBodies = 4 * nbMaxBodies;
        mBodies = new RigidBody[nbMaxBodies];
        mNbAllocatedBytesContactManifolds = 4 * nbMaxContactManifolds;
        mContactManifolds = new ContactManifold[nbMaxContactManifolds];
        mNbAllocatedBytesJoints = 4 * nbMaxJoints;
        mJoints = new Joint[nbMaxJoints];
    }

    // Add a body into the island
    public void addBody(RigidBody body) {
        assert (!body.isSleeping());
        mBodies[mNbBodies] = body;
        mNbBodies++;
    }

    // Add a contact manifold into the island
    public void addContactManifold(ContactManifold contactManifold) {
        mContactManifolds[mNbContactManifolds] = contactManifold;
        mNbContactManifolds++;
    }

    // Add a joint into the island
    public void addJoint(Joint joint) {
        mJoints[mNbJoints] = joint;
        mNbJoints++;
    }

    // Return the number of bodies in the island
    public int getNbBodies() {
        return mNbBodies;
    }

    // Return the number of contact manifolds in the island
    public int getNbContactManifolds() {
        return mNbContactManifolds;
    }

    // Return the number of joints in the island
    public int getNbJoints() {
        return mNbJoints;
    }

    // Return a pointer to the array of bodies
    public RigidBody[] getBodies() {
        return mBodies;
    }

    // Return a pointer to the array of contact manifolds
    public ContactManifold[] getContactManifold() {
        return mContactManifolds;
    }

    // Return a pointer to the array of joints
    public Joint[] getJoints() {
        return mJoints;
    }

}
