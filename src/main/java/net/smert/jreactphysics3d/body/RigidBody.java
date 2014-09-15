package net.smert.jreactphysics3d.body;

import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.constraint.Joint;
import net.smert.jreactphysics3d.constraint.JointListElement;
import net.smert.jreactphysics3d.engine.Material;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a rigid body of the physics engine. A rigid body is a non-deformable body that has a constant
 * mass. This class inherits from the CollisionBody class.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class RigidBody extends CollisionBody {

    // True if the gravity needs to be applied to this rigid body
    protected boolean mIsGravityEnabled;

    // Angular velocity damping factor
    protected float mAngularDamping;

    // Linear velocity damping factor
    protected float mLinearDamping;

    // TODO: Remove the mass variable (duplicate with inverseMass)
    // Mass of the body
    protected float mMass;

    // Inverse of the mass of the body
    protected float mMassInverse;

    // First element of the linked list of joints involving this body
    protected JointListElement mJointsList;

    // Angular velocity of the body
    protected final Vector3 mAngularVelocity;

    // Linear velocity of the body
    protected final Vector3 mLinearVelocity;

    // Current external force on the body
    protected final Vector3 mExternalForce;

    // Current external torque on the body
    protected final Vector3 mExternalTorque;

    // Material properties of the rigid body
    protected final Material mMaterial;

    // Local inertia tensor of the body (in local-space)
    protected final Matrix3x3 mInertiaTensorLocal;

    // Inverse of the inertia tensor of the body
    protected final Matrix3x3 mInertiaTensorLocalInverse;

    // Constructor
    public RigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal, CollisionShape collisionShape, int id) {
        super(transform, collisionShape, id);

        assert (mass > 0.0f);
        assert (inertiaTensorLocal != null);

        mIsGravityEnabled = true;
        mLinearDamping = 0.0f;
        mAngularDamping = 0.0f;
        mMass = mass;
        mMassInverse = 1.0f / mass;

        mJointsList = null;
        mAngularVelocity = new Vector3();
        mLinearVelocity = new Vector3();
        mExternalForce = new Vector3();
        mExternalTorque = new Vector3();
        mMaterial = new Material();
        mInertiaTensorLocal = new Matrix3x3(inertiaTensorLocal);
        mInertiaTensorLocalInverse = new Matrix3x3(inertiaTensorLocal).inverse();
    }

    // Return true if the gravity needs to be applied to this rigid body
    public boolean isGravityEnabled() {
        return mIsGravityEnabled;
    }

    // Set the variable to know if the gravity is applied to this rigid body
    public void enableGravity(boolean isEnabled) {
        mIsGravityEnabled = isEnabled;
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

    // Return the linear velocity damping factor
    public float getLinearDamping() {
        return mLinearDamping;
    }

    // Set the linear damping factor
    public void setLinearDamping(float linearDamping) {
        assert (linearDamping >= 0.0f);
        mLinearDamping = linearDamping;
    }

    // Method that return the mass of the body
    public float getMass() {
        return mMass;
    }

    // Method that set the mass of the body
    public void setMass(float mass) {
        assert (mass > 0.0f);

        // TODO: Set inverse mass when this is set?
        mMass = mass;
    }

    // Return the inverse of the mass of the body
    public float getMassInverse() {
        return mMassInverse;
    }

    // Set the inverse of the mass
    public void setMassInverse(float massInverse) {
        assert (massInverse >= 0.0f);
        mMassInverse = massInverse;
    }

    // Return the first element of the linked list of joints involving this body
    public JointListElement getJointsList() {
        return mJointsList;
    }

    public void setJointsList(JointListElement jointsList) {
        mJointsList = jointsList;
    }

    // Remove a joint from the joints list
    public void removeJointFromJointsList(Joint joint) {

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

    // Return the angular velocity of the body
    public Vector3 getAngularVelocity() {
        return mAngularVelocity;
    }

    public void setAngularVelocity(Vector3 angularVelocity) {
        mAngularVelocity.set(angularVelocity);
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
            mLinearVelocity.set(linearVelocity);
        }
    }

    public Vector3 getExternalForce() {
        return mExternalForce;
    }

    public Vector3 getExternalTorque() {
        return mExternalTorque;
    }

    // Apply an external force to the body at a given point (in world-space coordinates).
    // If the point is not at the center of gravity of the body, it will also
    // generate some torque and therefore, change the angular velocity of the body.
    // If the body is sleeping, calling this method will wake it up. Note that the
    // force will we added to the sum of the applied forces and that this sum will be
    // reset to zero at the end of each call of the DynamicsWorld::update() method.
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
        mExternalForce.add(force);
        mExternalTorque.add(new Vector3(point).subtract(mTransform.getPosition()).cross(force));
    }

    // Apply an external force to the body at its gravity center.
    // If the body is sleeping, calling this method will wake it up. Note that the
    // force will we added to the sum of the applied forces and that this sum will be
    // reset to zero at the end of each call of the DynamicsWorld::update() method.
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
        mExternalForce.add(force);
    }

    // Apply an external torque to the body.
    // If the body is sleeping, calling this method will wake it up. Note that the
    // force will we added to the sum of the applied torques and that this sum will be
    // reset to zero at the end of each call of the DynamicsWorld::update() method.
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
        mExternalTorque.add(torque);
    }

    // Return a reference to the material properties of the rigid body
    public Material getMaterial() {
        return mMaterial;
    }

    // Set a new material for this rigid body
    public void setMaterial(Material material) {
        mMaterial.set(material);
    }

    // Return the local inertia tensor of the body (in body coordinates)
    public Matrix3x3 getInertiaTensorLocal() {
        return mInertiaTensorLocal;
    }

    // Set the local inertia tensor of the body (in body coordinates)
    public void setInertiaTensorLocal(Matrix3x3 inertiaTensorLocal) {

        // TODO: Set inertiaTensorLocalInverse when this is set?
        mInertiaTensorLocal.set(inertiaTensorLocal);
    }

    // Return the inertia tensor in world coordinates.
    // The inertia tensor I_w in world coordinates is computed
    // with the local inertia tensor I_b in body coordinates
    // by I_w = R * I_b * R^T
    // where R is the rotation matrix (and R^T its transpose) of
    // the current orientation quaternion of the body
    public Matrix3x3 getInertiaTensorWorld() {
        // TODO: Rename to new
        Matrix3x3 rotation = mTransform.getOrientation().getMatrix(new Matrix3x3());
        Matrix3x3 transpose = new Matrix3x3(rotation).transpose();

        // Compute and return the inertia tensor in world coordinates
        return new Matrix3x3(rotation).multiply(mInertiaTensorLocal).multiply(transpose);
    }

    // Get the inverse of the inertia tensor
    public Matrix3x3 getInertiaTensorLocalInverse() {
        return mInertiaTensorLocalInverse;
    }

    // Return the inverse of the inertia tensor in world coordinates.
    // The inertia tensor I_w in world coordinates is computed with the
    // local inverse inertia tensor I_b^-1 in body coordinates
    // by I_w = R * I_b^-1 * R^T
    // where R is the rotation matrix (and R^T its transpose) of the
    // current orientation quaternion of the body
    public Matrix3x3 getInertiaTensorInverseWorld() {
        // TODO: Rename to new
        Matrix3x3 rotation = mTransform.getOrientation().getMatrix(new Matrix3x3());
        Matrix3x3 transpose = new Matrix3x3(rotation).transpose();

        // Compute and return the inertia tensor in world coordinates
        return new Matrix3x3(rotation).multiply(mInertiaTensorLocalInverse).multiply(transpose);
    }

    // Set the variable to know whether or not the body is sleeping
    @Override
    public void setIsSleeping(boolean isSleeping) {
        if (isSleeping) {
            mLinearVelocity.zero();
            mAngularVelocity.zero();
            mExternalForce.zero();
            mExternalTorque.zero();
        }
        super.setIsSleeping(isSleeping);
    }

}
