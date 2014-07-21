package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;
import net.smert.jreactphysics3d.memory.MemoryAllocator;

/**
 * This class represents the set of contact points between two bodies. The contact manifold is implemented in a way to
 * cache the contact points among the frames for better stability following the "Contact Generation" presentation of
 * Erwin Coumans at GDC 2010 conference (bullet.googlecode.com/files/GDC10_Coumans_Erwin_Contact.pdf). Some code of this
 * class is based on the implementation of the btPersistentManifold class from Bullet physics engine
 * (www.http://bulletphysics.org). The contacts between two bodies are added one after the other in the cache. When the
 * cache is full, we have to remove one point. The idea is to keep the point with the deepest penetration depth and also
 * to keep the points producing the larger area (for a more stable contact manifold). The new added point is always
 * kept.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactManifold {

    public static final int MAX_CONTACT_POINTS_IN_MANIFOLD = 4;

    /// Pointer to the first body of the contact
    private CollisionBody mBody1;

    /// Pointer to the second body of the contact
    private CollisionBody mBody2;

    /// Contact points in the manifold
    private ContactPoint[] mContactPoints = new ContactPoint[MAX_CONTACT_POINTS_IN_MANIFOLD];

    /// Number of contacts in the cache
    private int mNbContactPoints;

    /// First friction vector of the contact manifold
    private Vector3 mFrictionVector1;

    /// Second friction vector of the contact manifold
    private Vector3 mFrictionVector2;

    /// First friction raint accumulated impulse
    private float mFrictionImpulse1;

    /// Second friction raint accumulated impulse
    private float mFrictionImpulse2;

    /// Twist friction raint accumulated impulse
    private float mFrictionTwistImpulse;

    /// True if the contact manifold has already been added into an island
    private boolean mIsAlreadyInIsland;

    /// Reference to the memory allocator
    private MemoryAllocator mMemoryAllocator;

    // -------------------- Methods -------------------- //
    /// Private copy-constructor
    private ContactManifold(ContactManifold contactManifold) {
    }

    /// Private assignment operator
    private ContactManifold operatorEqual(ContactManifold contactManifold) {
        return this;
    }

    // Return a pointer to the first body of the contact manifold
    public CollisionBody getBody1() {
        return mBody1;
    }

    // Return a pointer to the second body of the contact manifold
    public CollisionBody getBody2() {
        return mBody2;
    }

    // Return the index of maximum area
    private int getMaxArea(float area0, float area1, float area2, float area3) {
        if (area0 < area1) {
            if (area1 < area2) {
                if (area2 < area3) {
                    return 3;
                } else {
                    return 2;
                }
            } else {
                if (area1 < area3) {
                    return 3;
                } else {
                    return 1;
                }
            }
        } else {
            if (area0 < area2) {
                if (area2 < area3) {
                    return 3;
                } else {
                    return 2;
                }
            } else {
                if (area0 < area3) {
                    return 3;
                } else {
                    return 0;
                }
            }
        }
    }

    // Return the index of the contact point with the larger penetration depth.
    /// This corresponding contact will be kept in the cache. The method returns -1 is
    /// the new contact is the deepest.
    private int getIndexOfDeepestPenetration(ContactPoint newContact) {
        assert (mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD);
        int indexMaxPenetrationDepth = -1;
        float maxPenetrationDepth = newContact.getPenetrationDepth();

        // For each contact in the cache
        for (int i = 0; i < mNbContactPoints; i++) {

            // If the current contact has a larger penetration depth
            if (mContactPoints[i].getPenetrationDepth() > maxPenetrationDepth) {
                maxPenetrationDepth = mContactPoints[i].getPenetrationDepth();
                indexMaxPenetrationDepth = i;
            }
        }

        // Return the index of largest penetration depth
        return indexMaxPenetrationDepth;
    }

    // Return the index that will be removed.
    /// The index of the contact point with the larger penetration
    /// depth is given as a parameter. This contact won't be removed. Given this contact, we compute
    /// the different area and we want to keep the contacts with the largest area. The new point is also
    /// kept. In order to compute the area of a quadrilateral, we use the formula :
    /// Area = 0.5 * | AC x BD | where AC and BD form the diagonals of the quadrilateral. Note that
    /// when we compute this area, we do not calculate it exactly but we
    /// only estimate it because we do not compute the actual diagonals of the quadrialteral. Therefore,
    /// this is only a guess that is faster to compute. This idea comes from the Bullet Physics library
    /// by Erwin Coumans (http://wwww.bulletphysics.org).
    private int getIndexToRemove(int indexMaxPenetration, Vector3 newPoint) {

        assert (mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD);

        float area0 = 0.0f;       // Area with contact 1,2,3 and newPoint
        float area1 = 0.0f;       // Area with contact 0,2,3 and newPoint
        float area2 = 0.0f;       // Area with contact 0,1,3 and newPoint
        float area3 = 0.0f;       // Area with contact 0,1,2 and newPoint

        if (indexMaxPenetration != 0) {
            // Compute the area
            Vector3 vector1 = Vector3.operatorSubtract(newPoint, mContactPoints[1].getLocalPointOnBody1());
            Vector3 vector2 = Vector3.operatorSubtract(mContactPoints[3].getLocalPointOnBody1(), mContactPoints[2].getLocalPointOnBody1());
            Vector3 crossProduct = vector1.cross(vector2);
            area0 = crossProduct.lengthSquare();
        }
        if (indexMaxPenetration != 1) {
            // Compute the area
            Vector3 vector1 = Vector3.operatorSubtract(newPoint, mContactPoints[0].getLocalPointOnBody1());
            Vector3 vector2 = Vector3.operatorSubtract(mContactPoints[3].getLocalPointOnBody1(), mContactPoints[2].getLocalPointOnBody1());
            Vector3 crossProduct = vector1.cross(vector2);
            area1 = crossProduct.lengthSquare();
        }
        if (indexMaxPenetration != 2) {
            // Compute the area
            Vector3 vector1 = Vector3.operatorSubtract(newPoint, mContactPoints[0].getLocalPointOnBody1());
            Vector3 vector2 = Vector3.operatorSubtract(mContactPoints[3].getLocalPointOnBody1(), mContactPoints[1].getLocalPointOnBody1());
            Vector3 crossProduct = vector1.cross(vector2);
            area2 = crossProduct.lengthSquare();
        }
        if (indexMaxPenetration != 3) {
            // Compute the area
            Vector3 vector1 = Vector3.operatorSubtract(newPoint, mContactPoints[0].getLocalPointOnBody1());
            Vector3 vector2 = Vector3.operatorSubtract(mContactPoints[2].getLocalPointOnBody1(), mContactPoints[1].getLocalPointOnBody1());
            Vector3 crossProduct = vector1.cross(vector2);
            area3 = crossProduct.lengthSquare();
        }

        // Return the index of the contact to remove
        return getMaxArea(area0, area1, area2, area3);
    }

    // Remove a contact point from the manifold
    private void removeContactPoint(int index) {
        assert (index < mNbContactPoints);
        assert (mNbContactPoints > 0);

        // Call the destructor explicitly and tell the memory allocator that
        // the corresponding memory block is now free
        //mContactPoints[index].ContactPoint::~ContactPoint();
        //mMemoryAllocator.release(mContactPoints[index], sizeof(ContactPoint));
        // If we don't remove the last index
        if (index < mNbContactPoints - 1) {
            mContactPoints[index] = mContactPoints[mNbContactPoints - 1];
        }

        mNbContactPoints--;
    }

    // Constructor
    public ContactManifold(CollisionBody body1, CollisionBody body2, MemoryAllocator memoryAllocator) {
        mBody1 = body1;
        mBody2 = body2;
        mNbContactPoints = 0;
        mFrictionImpulse1 = 0.0f;
        mFrictionImpulse2 = 0.0f;
        mFrictionTwistImpulse = 0.0f;
        mIsAlreadyInIsland = false;
        mMemoryAllocator = memoryAllocator;
    }

    // Return the number of contact points in the manifold
    public int getNbContactPoints() {
        return mNbContactPoints;
    }

    // Return the first friction vector at the center of the contact manifold
    public Vector3 getFrictionVector1() {
        return mFrictionVector1;
    }

    // set the first friction vector at the center of the contact manifold
    public void setFrictionVector1(Vector3 frictionVector1) {
        mFrictionVector1 = frictionVector1;
    }

    // Return the second friction vector at the center of the contact manifold
    public Vector3 getFrictionVector2() {
        return mFrictionVector2;
    }

    // set the second friction vector at the center of the contact manifold
    public void setFrictionVector2(Vector3 frictionVector2) {
        mFrictionVector2 = frictionVector2;
    }

    // Return the first friction accumulated impulse
    public float getFrictionImpulse1() {
        return mFrictionImpulse1;
    }

    // Set the first friction accumulated impulse
    public void setFrictionImpulse1(float frictionImpulse1) {
        mFrictionImpulse1 = frictionImpulse1;
    }

    // Return the second friction accumulated impulse
    public float getFrictionImpulse2() {
        return mFrictionImpulse2;
    }

    // Set the second friction accumulated impulse
    public void setFrictionImpulse2(float frictionImpulse2) {
        mFrictionImpulse2 = frictionImpulse2;
    }

    // Return the friction twist accumulated impulse
    public float getFrictionTwistImpulse() {
        return mFrictionTwistImpulse;
    }

    // Set the friction twist accumulated impulse
    public void setFrictionTwistImpulse(float frictionTwistImpulse) {
        mFrictionTwistImpulse = frictionTwistImpulse;
    }

    // Return a contact point of the manifold
    public ContactPoint getContactPoint(int index) {
        assert (index >= 0 && index < mNbContactPoints);
        return mContactPoints[index];
    }

    // Return true if the contact manifold has already been added into an island
    public boolean isAlreadyInIsland() {
        return mIsAlreadyInIsland;
    }

    public void setIsAlreadyInIsland(boolean isAlreadyInIsland) {
        mIsAlreadyInIsland = isAlreadyInIsland;
    }

    // Add a contact point in the manifold
    public void addContactPoint(ContactPoint contact) {

        // For contact already in the manifold
        for (int i = 0; i < mNbContactPoints; i++) {

            // Check if the new point point does not correspond to a same contact point
            // already in the manifold.
            float distance = Vector3.operatorSubtract(mContactPoints[i].getWorldPointOnBody1(), contact.getWorldPointOnBody1()).lengthSquare();
            if (distance <= Defaults.PERSISTENT_CONTACT_DIST_THRESHOLD * Defaults.PERSISTENT_CONTACT_DIST_THRESHOLD) {

                // Delete the new contact
                //contact.ContactPoint::~ContactPoint();
                //mMemoryAllocator.release(contact, sizeof(ContactPoint));
                //removeContact(i);
                return;
                //break;
            }
        }

        // If the contact manifold is full
        if (mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD) {
            int indexMaxPenetration = getIndexOfDeepestPenetration(contact);
            int indexToRemove = getIndexToRemove(indexMaxPenetration, contact.getLocalPointOnBody1());
            removeContactPoint(indexToRemove);
        }

        // Add the new contact point in the manifold
        mContactPoints[mNbContactPoints] = contact;
        mNbContactPoints++;
    }

    // Update the contact manifold
    /// First the world space coordinates of the current contacts in the manifold are recomputed from
    /// the corresponding transforms of the bodies because they have moved. Then we remove the contacts
    /// with a negative penetration depth (meaning that the bodies are not penetrating anymore) and also
    /// the contacts with a too large distance between the contact points in the plane orthogonal to the
    /// contact normal.
    public void update(Transform transform1, Transform transform2) {

        if (mNbContactPoints == 0) {
            return;
        }

        // Update the world coordinates and penetration depth of the contact points in the manifold
        for (int i = 0; i < mNbContactPoints; i++) {
            mContactPoints[i].setWorldPointOnBody1(transform1.operatorMultiply(mContactPoints[i].getLocalPointOnBody1()));
            mContactPoints[i].setWorldPointOnBody2(transform2.operatorMultiply(mContactPoints[i].getLocalPointOnBody2()));
            mContactPoints[i].setPenetrationDepth(Vector3.operatorSubtract(mContactPoints[i].getWorldPointOnBody1(),
                    mContactPoints[i].getWorldPointOnBody2()).dot(mContactPoints[i].getNormal()));
        }

        float squarePersistentContactThreshold = Defaults.PERSISTENT_CONTACT_DIST_THRESHOLD * Defaults.PERSISTENT_CONTACT_DIST_THRESHOLD;

        // Remove the contact points that don't represent very well the contact manifold
        for (int i = (int) (mNbContactPoints) - 1; i >= 0; i--) {
            assert (i < (int) (mNbContactPoints));

            // Compute the distance between contact points in the normal direction
            float distanceNormal = -mContactPoints[i].getPenetrationDepth();

            // If the contacts points are too far from each other in the normal direction
            if (distanceNormal > squarePersistentContactThreshold) {
                removeContactPoint(i);
            } else {
                // Compute the distance of the two contact points in the plane
                // orthogonal to the contact normal
                Vector3 projOfPoint1 = Vector3.operatorAdd(mContactPoints[i].getWorldPointOnBody1(),
                        Vector3.operatorMultiply(mContactPoints[i].getNormal(), distanceNormal));
                Vector3 projDifference = Vector3.operatorSubtract(mContactPoints[i].getWorldPointOnBody2(), projOfPoint1);

                // If the orthogonal distance is larger than the valid distance
                // threshold, we remove the contact
                if (projDifference.lengthSquare() > squarePersistentContactThreshold) {
                    removeContactPoint(i);
                }
            }
        }
    }

    // Clear the contact manifold
    public void clear() {
        for (int i = 0; i < mNbContactPoints; i++) {

            // Call the destructor explicitly and tell the memory allocator that
            // the corresponding memory block is now free
            //mContactPoints[i].ContactPoint::~ContactPoint();
            //mMemoryAllocator.release(mContactPoints[i], sizeof(ContactPoint));
        }
        mNbContactPoints = 0;
    }

}
