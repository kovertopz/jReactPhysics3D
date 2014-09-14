package net.smert.jreactphysics3d.body;

import net.smert.jreactphysics3d.collision.shapes.AABB;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.engine.ContactManifoldListElement;
import net.smert.jreactphysics3d.mathematics.Transform;

/**
 * This class represents a body that is able to collide with others bodies. This class inherits from the Body class.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CollisionBody extends Body {

    // True if the body has moved during the last frame
    protected boolean mHasMoved;

    // True if the body can collide with others bodies
    protected boolean mIsCollisionEnabled;

    // True if the body is able to move
    protected boolean mIsMotionEnabled;

    // Interpolation factor used for the state interpolation
    protected float mInterpolationFactor;

    // AABB for Broad-Phase collision detection
    protected final AABB mAabb;

    // Collision shape of the body
    protected CollisionShape mCollisionShape;

    // First element of the linked list of contact manifolds involving this body
    protected ContactManifoldListElement mContactManifoldsList;

    // Last position and orientation of the body
    protected final Transform mOldTransform;

    // Position and orientation of the body
    protected final Transform mTransform;

    // Constructor
    public CollisionBody(Transform transform, CollisionShape collisionShape, int id) {
        super(id);

        assert (transform != null);
        assert (collisionShape != null);

        mHasMoved = false;
        mIsCollisionEnabled = true;
        mIsMotionEnabled = true;
        mInterpolationFactor = 0.0f;

        mAabb = new AABB();
        mCollisionShape = collisionShape;
        mContactManifoldsList = null;
        mOldTransform = new Transform(transform);
        mTransform = new Transform(transform);

        // Initialize the AABB for broad-phase collision detection
        mCollisionShape.updateAABB(mAabb, transform);
    }

    public boolean getHasMoved() {
        return mHasMoved;
    }

    public void setHasMoved(boolean hasMoved) {
        mHasMoved = hasMoved;
    }

    // Return true if the body can collide with others bodies
    public boolean isCollisionEnabled() {
        return mIsCollisionEnabled;
    }

    // Enable/disable the collision with this body
    public void setIsCollisionEnabled(boolean isCollisionEnabled) {
        mIsCollisionEnabled = isCollisionEnabled;
    }

    // Return true if the rigid body is allowed to move
    public boolean isMotionEnabled() {
        return mIsMotionEnabled;
    }

    // Enable/disable the motion of the body
    public void setIsMotionEnabled(boolean isMotionEnabled) {
        mIsMotionEnabled = isMotionEnabled;
    }

    // Set the interpolation factor of the body
    public void setInterpolationFactor(float interpolationFactor) {
        mInterpolationFactor = interpolationFactor;
    }

    // Return the AAABB of the body
    public AABB getAABB() {
        return mAabb;
    }

    // Update the rigid body in order to reflect a change in the body state
    public void updateAABB() {
        mCollisionShape.updateAABB(mAabb, mTransform);
    }

    // Return the collision shape
    public CollisionShape getCollisionShape() {
        return mCollisionShape;
    }

    // Set the collision shape
    public void setCollisionShape(CollisionShape collisionShape) {

        // TODO: Should we even be allowed to change collision shapes?
        assert (collisionShape != null);
        mCollisionShape = collisionShape;
    }

    // Return the first element of the linked list of contact manifolds involving this body
    public ContactManifoldListElement getContactManifoldsLists() {
        return mContactManifoldsList;
    }

    public void setContactManifoldsLists(ContactManifoldListElement contactManifoldsList) {
        mContactManifoldsList = contactManifoldsList;
    }

    // Reset the contact manifold lists
    public void resetContactManifoldsList() {
        mContactManifoldsList = null;
        assert (mContactManifoldsList == null);
    }

    // Return the interpolated transform for rendering
    public Transform getInterpolatedTransform(Transform outTransform) {
        return Transform.Interpolate(mOldTransform, mTransform, mInterpolationFactor, outTransform);
    }

    // Update the old transform with the current one.
    // This is used to compute the interpolated position and orientation of the body
    public void updateOldTransform() {
        mOldTransform.set(mTransform);
    }

    // Return the current position and orientation
    public Transform getTransform() {
        return mTransform;
    }

    // Set the current position and orientation
    public void setTransform(Transform transform) {

        // Check if the body has moved
        if (!mTransform.equals(transform)) {
            mHasMoved = true;
        }

        mTransform.set(transform);
    }

}
