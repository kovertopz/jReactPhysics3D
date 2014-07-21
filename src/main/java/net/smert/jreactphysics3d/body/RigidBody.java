package net.smert.jreactphysics3d.body;

import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.constraint.Joint;
import net.smert.jreactphysics3d.constraint.JointListElement;
import net.smert.jreactphysics3d.engine.Material;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;
import net.smert.jreactphysics3d.memory.MemoryAllocator;

/**
 * This class represents a rigid body of the physics engine. A rigid body is a non-deformable body that has a constant
 * mass. This class inherits from the CollisionBody class.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class RigidBody extends CollisionBody {

    // TODO : Remove the mass variable (duplicate with inverseMass)
    /// Mass of the body
    protected float mMass;

    /// Linear velocity of the body
    protected Vector3 mLinearVelocity;

    /// Angular velocity of the body
    protected Vector3 mAngularVelocity;

    /// Current external force on the body
    protected Vector3 mExternalForce;

    /// Current external torque on the body
    protected Vector3 mExternalTorque;

    /// Local inertia tensor of the body (in local-space)
    protected Matrix3x3 mInertiaTensorLocal;

    /// Inverse of the inertia tensor of the body
    protected Matrix3x3 mInertiaTensorLocalInverse;

    /// Inverse of the mass of the body
    protected float mMassInverse;

    /// True if the gravity needs to be applied to this rigid body
    protected boolean mIsGravityEnabled;

    /// Material properties of the rigid body
    protected Material mMaterial;

    /// Linear velocity damping factor
    protected float mLinearDamping;

    /// Angular velocity damping factor
    protected float mAngularDamping;

    /// First element of the linked list of joints involving this body
    protected JointListElement mJointsList;

    // Constructor
    public RigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal, CollisionShape collisionShape, int id) {
        super(transform, collisionShape, id);

        assert (transform != null);
        assert (inertiaTensorLocal != null);
        assert (collisionShape != null);

        mMass = mass;
        mLinearVelocity = new Vector3();
        mAngularVelocity = new Vector3();
        mExternalForce = new Vector3();
        mExternalTorque = new Vector3();
        mInertiaTensorLocal = inertiaTensorLocal;
        mInertiaTensorLocalInverse = inertiaTensorLocal.getInverse();
        mMassInverse = 1.0f / mass;
        mIsGravityEnabled = true;
        mMaterial = new Material();
        mLinearDamping = 0.0f;
        mAngularDamping = 0.0f;
        mJointsList = null;
    }

    // Method that return the mass of the body
    public float getMass() {
        return mMass;
    }

    // Method that set the mass of the body
    public void setMass(float mass) {

        // TODO: Set inverse mass when this is set?
        mMass = mass;
    }

    // Return the linear velocity
    public Vector3 getLinearVelocity() {
        return mLinearVelocity;
    }

    // Set the linear velocity of the rigid body
    public void setLinearVelocity(Vector3 linearVelocity) {

        // If the body is able to move
        if (mIsMotionEnabled) {
            // Update the linear velocity of the current body state
            mLinearVelocity = linearVelocity;
        }
    }

    // Return the angular velocity of the body
    public Vector3 getAngularVelocity() {
        return mAngularVelocity;
    }

    public void setAngularVelocity(Vector3 angularVelocity) {
        mAngularVelocity = angularVelocity;
    }

    // Apply an external force to the body at a given point (in world-space coordinates).
    /// If the point is not at the center of gravity of the body, it will also
    /// generate some torque and therefore, change the angular velocity of the body.
    /// If the body is sleeping, calling this method will wake it up. Note that the
    /// force will we added to the sum of the applied forces and that this sum will be
    /// reset to zero at the end of each call of the DynamicsWorld::update() method.
    public void applyForce(Vector3 force, Vector3 point) {

        // If it is a static body, do not apply any force
        if (!mIsMotionEnabled) {
            return;
        }

        // Awake the body if it was sleeping
        if (mIsSleeping) {
            setIsSleeping(false);
        }

        // Add the force and torque
        mExternalForce.operatorAddEqual(force);
        mExternalTorque.operatorAddEqual(Vector3.operatorSubtract(point, mTransform.getPosition()).cross(force));
    }

    // Apply an external force to the body at its gravity center.
    /// If the body is sleeping, calling this method will wake it up. Note that the
    /// force will we added to the sum of the applied forces and that this sum will be
    /// reset to zero at the end of each call of the DynamicsWorld::update() method.
    public void applyForceToCenter(Vector3 force) {
        // If it is a static body, do not apply any force
        if (!mIsMotionEnabled) {
            return;
        }

        // Awake the body if it was sleeping
        if (mIsSleeping) {
            setIsSleeping(false);
        }

        // Add the force
        mExternalForce.operatorAddEqual(force);
    }

    // Apply an external torque to the body.
    /// If the body is sleeping, calling this method will wake it up. Note that the
    /// force will we added to the sum of the applied torques and that this sum will be
    /// reset to zero at the end of each call of the DynamicsWorld::update() method.
    public void applyTorque(Vector3 torque) {

        // If it is a static body, do not apply any force
        if (!mIsMotionEnabled) {
            return;
        }

        // Awake the body if it was sleeping
        if (mIsSleeping) {
            setIsSleeping(false);
        }

        // Add the torque
        mExternalTorque.operatorAddEqual(torque);
    }

    // Return the local inertia tensor of the body (in body coordinates)
    public Matrix3x3 getInertiaTensorLocal() {
        return mInertiaTensorLocal;
    }

    // Set the local inertia tensor of the body (in body coordinates)
    public void setInertiaTensorLocal(Matrix3x3 inertiaTensorLocal) {

        // TODO: Set inertiaTensorLocalInverse when this is set?
        mInertiaTensorLocal = inertiaTensorLocal;
    }

    // Return the inertia tensor in world coordinates.
    /// The inertia tensor I_w in world coordinates is computed
    /// with the local inertia tensor I_b in body coordinates
    /// by I_w = R * I_b * R^T
    /// where R is the rotation matrix (and R^T its transpose) of
    /// the current orientation quaternion of the body
    public Matrix3x3 getInertiaTensorWorld() {

        // TODO: Optimize
        // Compute and return the inertia tensor in world coordinates
        return Matrix3x3.operatorMultiply(mTransform.getOrientation().getMatrix(), mInertiaTensorLocal)
                .operatorSubtractEqual(mTransform.getOrientation().getMatrix().getTranspose());
    }

    // Get the inverse of the inertia tensor
    public Matrix3x3 getInertiaTensorLocalInverse() {
        return mInertiaTensorLocalInverse;
    }

    // Return the inverse of the inertia tensor in world coordinates.
    /// The inertia tensor I_w in world coordinates is computed with the
    /// local inverse inertia tensor I_b^-1 in body coordinates
    /// by I_w = R * I_b^-1 * R^T
    /// where R is the rotation matrix (and R^T its transpose) of the
    /// current orientation quaternion of the body
    public Matrix3x3 getInertiaTensorInverseWorld() {

        // TODO: Optimize
        // Compute and return the inertia tensor in world coordinates
        return Matrix3x3.operatorMultiply(mTransform.getOrientation().getMatrix(), mInertiaTensorLocalInverse)
                .operatorSubtractEqual(mTransform.getOrientation().getMatrix().getTranspose());
    }

    // Return the inverse of the mass of the body
    public float getMassInverse() {
        return mMassInverse;
    }

    // Set the inverse of the mass
    public void setMassInverse(float massInverse) {
        mMassInverse = massInverse;
    }

    // Return true if the gravity needs to be applied to this rigid body
    public boolean isGravityEnabled() {
        return mIsGravityEnabled;
    }

    // Set the variable to know if the gravity is applied to this rigid body
    public void enableGravity(boolean isEnabled) {
        mIsGravityEnabled = isEnabled;
    }

    // Return a reference to the material properties of the rigid body
    public Material getMaterial() {
        return mMaterial;
    }

    // Set a new material for this rigid body
    public void setMaterial(Material material) {
        mMaterial = material;
    }

    // Return the linear velocity damping factor
    public float getLinearDamping() {
        return mLinearDamping;
    }

    // Set the linear damping factor
    public void setLinearDamping(float linearDamping) {
        assert (linearDamping >= 0.0f);
        mLinearDamping = linearDamping;
    }

    // Return the angular velocity damping factor
    public float getAngularDamping() {
        return mAngularDamping;
    }

    // Set the angular damping factor
    public void setAngularDamping(float angularDamping) {
        assert (angularDamping >= 0.0f);
        mAngularDamping = angularDamping;
    }

    // Return the first element of the linked list of joints involving this body
    public JointListElement getJointsList() {
        return mJointsList;
    }

    // Remove a joint from the joints list
    public void removeJointFromJointsList(MemoryAllocator memoryAllocator, Joint joint) {

        assert (joint != null);
        assert (mJointsList != null);

        // Remove the joint from the linked list of the joints of the first body
        if (mJointsList.joint == joint) {   // If the first element is the one to remove
            JointListElement elementToRemove = mJointsList;
            mJointsList = elementToRemove.next;
        } else {    // If the element to remove is not the first one in the list
            JointListElement currentElement = mJointsList;
            while (currentElement.next != null) {
                if (currentElement.next.joint == joint) {
                    JointListElement elementToRemove = currentElement.next;
                    currentElement.next = elementToRemove.next;
                    break;
                }
                currentElement = currentElement.next;
            }
        }
    }

    // Set the variable to know whether or not the body is sleeping
    @Override
    public void setIsSleeping(boolean isSleeping) {

        if (isSleeping) {
            mLinearVelocity.setToZero();
            mAngularVelocity.setToZero();
            mExternalForce.setToZero();
            mExternalTorque.setToZero();
        }

        super.setIsSleeping(isSleeping);
    }

}
