/*
 * ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/
 * Copyright (c) 2010-2013 Daniel Chappuis
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim
 *    that you wrote the original software. If you use this software in a
 *    product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * This file has been modified during the port to Java and differ from the source versions.
 */
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
    protected boolean hasMoved;

    // True if the body can collide with others bodies
    protected boolean isCollisionEnabled;

    // True if the body is able to move
    protected boolean isMotionEnabled;

    // Interpolation factor used for the state interpolation
    protected float interpolationFactor;

    // AABB for Broad-Phase collision detection
    protected final AABB aabb;

    // Collision shape of the body
    protected CollisionShape collisionShape;

    // First element of the linked list of contact manifolds involving this body
    protected ContactManifoldListElement contactManifoldsList;

    // Last position and orientation of the body
    protected final Transform oldTransform;

    // Position and orientation of the body
    protected final Transform transform;

    // Constructor
    public CollisionBody(Transform transform, CollisionShape collisionShape, int id) {
        super(id);

        assert (transform != null);
        assert (collisionShape != null);

        hasMoved = false;
        isCollisionEnabled = true;
        isMotionEnabled = true;
        interpolationFactor = 0.0f;
        aabb = new AABB();
        this.collisionShape = collisionShape;
        contactManifoldsList = null;
        oldTransform = new Transform(transform);
        this.transform = new Transform(transform);

        // Initialize the AABB for broad-phase collision detection
        this.collisionShape.updateAABB(aabb, transform);
    }

    public boolean getHasMoved() {
        return hasMoved;
    }

    public void setHasMoved(boolean hasMoved) {
        this.hasMoved = hasMoved;
    }

    // Return true if the body can collide with others bodies
    public boolean isCollisionEnabled() {
        return isCollisionEnabled;
    }

    // Enable/disable the collision with this body
    public void setIsCollisionEnabled(boolean isCollisionEnabled) {
        this.isCollisionEnabled = isCollisionEnabled;
    }

    // Return true if the rigid body is allowed to move
    public boolean isMotionEnabled() {
        return isMotionEnabled;
    }

    // Enable/disable the motion of the body
    public void setIsMotionEnabled(boolean isMotionEnabled) {
        this.isMotionEnabled = isMotionEnabled;
    }

    // Set the interpolation factor of the body
    public void setInterpolationFactor(float interpolationFactor) {
        this.interpolationFactor = interpolationFactor;
    }

    // Return the AAABB of the body
    public AABB getAABB() {
        return aabb;
    }

    // Update the rigid body in order to reflect a change in the body state
    public void updateAABB() {
        collisionShape.updateAABB(aabb, transform);
    }

    // Return the collision shape
    public CollisionShape getCollisionShape() {
        return collisionShape;
    }

    // Set the collision shape
    public void setCollisionShape(CollisionShape collisionShape) {

        // TODO: Should we even be allowed to change collision shapes?
        assert (collisionShape != null);
        this.collisionShape = collisionShape;
    }

    // Return the first element of the linked list of contact manifolds involving this body
    public ContactManifoldListElement getContactManifoldsLists() {
        return contactManifoldsList;
    }

    public void setContactManifoldsLists(ContactManifoldListElement contactManifoldsList) {
        this.contactManifoldsList = contactManifoldsList;
    }

    // Reset the contact manifold lists
    public void resetContactManifoldsList() {
        contactManifoldsList = null;
        assert (contactManifoldsList == null);
    }

    // Return the interpolated transform for rendering
    public Transform getInterpolatedTransform(Transform outTransform) {
        return Transform.Interpolate(oldTransform, transform, interpolationFactor, outTransform);
    }

    // Update the old transform with the current one.
    // This is used to compute the interpolated position and orientation of the body
    public void updateOldTransform() {
        oldTransform.set(transform);
    }

    // Return the current position and orientation
    public Transform getTransform() {
        return transform;
    }

    // Set the current position and orientation
    public void setTransform(Transform transform) {

        // Check if the body has moved
        if (!this.transform.equals(transform)) {
            hasMoved = true;
        }

        this.transform.set(transform);
    }

}
