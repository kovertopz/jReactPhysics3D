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

    // Constructor
    public CollisionBody(Transform transform, CollisionShape collisionShape, int id) {
        super(id);

        assert (transform != null);
        assert (collisionShape != null);

        mCollisionShape = collisionShape;
        mTransform = transform;
        // Initialize the old transform
        mOldTransform = transform;
        mInterpolationFactor = 0.0f;
        mIsMotionEnabled = true;
        mIsCollisionEnabled = true;
        mAabb = new AABB();
        mHasMoved = false;
        mContactManifoldsList = null;

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
        mCollisionShape = collisionShape;
        assert (mCollisionShape != null);
    }

    // Return the interpolated transform for rendering
    public Transform getInterpolatedTransform() {
        return Transform.interpolateTransforms(mOldTransform, mTransform, mInterpolationFactor);
    }

    // Update the old transform with the current one.
    /// This is used to compute the interpolated position and orientation of the body
    public void updateOldTransform() {
        mOldTransform = mTransform;
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
        if (!mTransform.equals(transform)) {
            mHasMoved = true;
        } // TODO: Reset when not equal. Report bug to upstream.

        mTransform = transform;
    }

    // Return the AAABB of the body
    public AABB getAABB() {
        return mAabb;
    }

    // Update the rigid body in order to reflect a change in the body state
    public void updateAABB() {

        // TODO: Update only when has moved. Report bug to upsteam.
        // Update the AABB
        mCollisionShape.updateAABB(mAabb, mTransform);
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

    // Reset the contact manifold lists
    public void resetContactManifoldsList() {
        mContactManifoldsList = null;

        assert (mContactManifoldsList == null);
    }

}
