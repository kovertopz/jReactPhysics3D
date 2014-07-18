package net.smert.jreactphysics3d.body;

import net.smert.jreactphysics3d.collision.shapes.AABB;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.engine.ContactManifoldListElement;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.memory.MemoryAllocator;

/**
 * This class represents a body that is able to collide with others bodies. This
 * class inherits from the Body class.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CollisionBody extends Body {

    /// Collision shape of the body
    protected CollisionShape mCollisionShape;

    /// Position and orientation of the body
    protected Transform mTransform;

    /// Last position and orientation of the body
    protected Transform mOldTransform;

    /// Interpolation factor used for the state interpolation
    protected float mInterpolationFactor;

    /// True if the body is able to move
    protected boolean mIsMotionEnabled;

    /// True if the body can collide with others bodies
    protected boolean mIsCollisionEnabled;

    /// AABB for Broad-Phase collision detection
    protected AABB mAabb;

    /// True if the body has moved during the last frame
    protected boolean mHasMoved;

    /// First element of the linked list of contact manifolds involving this body
    protected ContactManifoldListElement mContactManifoldsList;

    /// Private copy-constructor
    protected CollisionBody(CollisionBody body) {
        super(body);
    }

    /// Private assignment operator
    protected CollisionBody operatorEqual(CollisionBody body) {
        return null;
    }

    // Reset the contact manifold lists
    protected void resetContactManifoldsList(MemoryAllocator memoryAllocator) {

        // Delete the linked list of contact manifolds of that body
        ContactManifoldListElement currentElement = mContactManifoldsList;
        while (currentElement != null) {
            ContactManifoldListElement nextElement = currentElement.next;

            // Delete the current element
            //currentElement.ContactManifoldListElement::~ContactManifoldListElement();
            //memoryAllocator.release(currentElement, sizeof(ContactManifoldListElement));
            currentElement = nextElement;
        }
        mContactManifoldsList = null;

        assert (mContactManifoldsList == null);
    }

    // Update the old transform with the current one.
    /// This is used to compute the interpolated position and orientation of the body
    protected void updateOldTransform() {
        mOldTransform = mTransform;
    }

    // Update the rigid body in order to reflect a change in the body state
    protected void updateAABB() {

        // Update the AABB
        mCollisionShape.updateAABB(mAabb, mTransform);
    }

    // Constructor
    public CollisionBody(Transform transform, CollisionShape collisionShape, int id) {
        super(id);

        assert (collisionShape != null);

        mCollisionShape = collisionShape;
        mTransform = transform;
        mHasMoved = false;
        mContactManifoldsList = null;

        mIsMotionEnabled = true;
        mIsCollisionEnabled = true;
        mInterpolationFactor = 0.0f;

        // Initialize the old transform
        mOldTransform = transform;

        // Initialize the AABB for broad-phase collision detection
        mCollisionShape.updateAABB(mAabb, transform);
    }

    // Return the collision shape
    public CollisionShape getCollisionShape() {
        assert (mCollisionShape != null);
        return mCollisionShape;
    }

    // Set the collision shape
    public void setCollisionShape(CollisionShape collisionShape) {
        assert (collisionShape != null);
        mCollisionShape = collisionShape;
    }

    // Return the interpolated transform for rendering
    public Transform getInterpolatedTransform() {
        return Transform.interpolateTransforms(mOldTransform, mTransform, mInterpolationFactor);
    }

    // Set the interpolation factor of the body
    public void setInterpolationFactor(float factor) {
        // Set the factor
        mInterpolationFactor = factor;
    }

    // Return true if the rigid body is allowed to move
    public boolean isMotionEnabled() {
        return mIsMotionEnabled;
    }

    // Enable/disable the motion of the body
    public void enableMotion(boolean isMotionEnabled) {
        mIsMotionEnabled = isMotionEnabled;
    }

    // Return the current position and orientation
    public Transform getTransform() {
        return mTransform;
    }

    // Set the current position and orientation
    public void setTransform(Transform transform) {

        // Check if the body has moved
        if (mTransform != transform) {
            mHasMoved = true;
        }

        mTransform = transform;
    }

    // Return the AAABB of the body
    public AABB getAABB() {
        return mAabb;
    }

    // Return true if the body can collide with others bodies
    public boolean isCollisionEnabled() {
        return mIsCollisionEnabled;
    }

    // Enable/disable the collision with this body
    public void enableCollision(boolean isCollisionEnabled) {
        mIsCollisionEnabled = isCollisionEnabled;
    }

    // Return the first element of the linked list of contact manifolds involving this body
    public ContactManifoldListElement getContactManifoldsLists() {
        return mContactManifoldsList;
    }

}
