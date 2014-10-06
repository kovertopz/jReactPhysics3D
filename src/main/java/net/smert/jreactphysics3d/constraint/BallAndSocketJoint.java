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
package net.smert.jreactphysics3d.constraint;

import net.smert.jreactphysics3d.configuration.JointsPositionCorrectionTechnique;
import net.smert.jreactphysics3d.engine.ConstraintSolverData;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation between two bodies. This joint has three
 * degrees of freedom. It can be used to create a chain of bodies for instance.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BallAndSocketJoint extends Joint {

    // Beta value for the bias factor of position correction
    private static final float BETA = 0.2f;

    // Anchor point of body 1 (in local-space coordinates of body 1)
    private Vector3 mLocalAnchorPointBody1;

    // Anchor point of body 2 (in local-space coordinates of body 2)
    private Vector3 mLocalAnchorPointBody2;

    // Vector from center of body 2 to anchor point in world-space
    private Vector3 mR1World;

    // Vector from center of body 2 to anchor point in world-space
    private Vector3 mR2World;

    // Inertia tensor of body 1 (in world-space coordinates)
    private Matrix3x3 mI1;

    // Inertia tensor of body 2 (in world-space coordinates)
    private Matrix3x3 mI2;

    // Bias vector for the constraint
    private Vector3 mBiasVector;

    // Inverse mass matrix K=JM^-1J^-t of the constraint
    private Matrix3x3 mInverseMassMatrix;

    // Accumulated impulse
    private Vector3 mImpulse;

    // Constructor
    public BallAndSocketJoint(BallAndSocketJointInfo jointInfo) {
        super(jointInfo);

        mImpulse = new Vector3();

        // Compute the local-space anchor point for each body
        mLocalAnchorPointBody1 = new Transform(mBody1.getTransform()).inverse().multiply(jointInfo.anchorPointWorldSpace, new Vector3());
        mLocalAnchorPointBody2 = new Transform(mBody2.getTransform()).inverse().multiply(jointInfo.anchorPointWorldSpace, new Vector3());
    }

    // Initialize before solving the constraint
    @Override
    public void initBeforeSolve(ConstraintSolverData constraintSolverData) {

        // Initialize the bodies index in the velocity array
        mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.get(mBody1);
        mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.get(mBody2);

        // Get the bodies positions and orientations
        Vector3 x1 = mBody1.getTransform().getPosition();
        Vector3 x2 = mBody2.getTransform().getPosition();
        Quaternion orientationBody1 = mBody1.getTransform().getOrientation();
        Quaternion orientationBody2 = mBody2.getTransform().getOrientation();

        // Get the inertia tensor of bodies
        mI1 = mBody1.getInertiaTensorInverseWorld();
        mI2 = mBody2.getInertiaTensorInverseWorld();

        // Compute the vector from body center to the anchor point in world-space
        orientationBody1.multiply(mLocalAnchorPointBody1, mR1World);
        orientationBody2.multiply(mLocalAnchorPointBody2, mR2World);

        // Compute the corresponding skew-symmetric matrices
        Matrix3x3 skewSymmetricMatrixU1 = new Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(mR1World);
        Matrix3x3 skewSymmetricMatrixU2 = new Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(mR2World);

        // Compute the matrix K=JM^-1J^t (3x3 matrix)
        float inverseMassBodies = 0.0f;
        if (mBody1.isMotionEnabled()) {
            inverseMassBodies += mBody1.getMassInverse();
        }
        if (mBody2.isMotionEnabled()) {
            inverseMassBodies += mBody2.getMassInverse();
        }
        Matrix3x3 massMatrix = new Matrix3x3(inverseMassBodies, 0.0f, 0.0f,
                0.0f, inverseMassBodies, 0.0f,
                0.0f, 0.0f, inverseMassBodies);
        if (mBody1.isMotionEnabled()) {
            massMatrix.add(
                    new Matrix3x3(skewSymmetricMatrixU1).multiply(new Matrix3x3(mI1).multiply(new Matrix3x3(skewSymmetricMatrixU1).transpose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.add(
                    new Matrix3x3(skewSymmetricMatrixU2).multiply(new Matrix3x3(mI2).multiply(new Matrix3x3(skewSymmetricMatrixU2).transpose())));
        }

        // Compute the inverse mass matrix K^-1
        mInverseMassMatrix.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrix = new Matrix3x3(massMatrix).inverse();
        }

        // Compute the bias "b" of the constraint
        mBiasVector.zero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            float biasFactor = (BETA / constraintSolverData.timeStep);
            mBiasVector = new Vector3(
                    new Vector3(new Vector3(new Vector3(x2).add(mR2World)).subtract(x1)).subtract(mR1World)).multiply(biasFactor);
        }

        // If warm-starting is not enabled
        if (!constraintSolverData.isWarmStartingActive) {

            // Reset the accumulated impulse
            mImpulse.zero();
        }
    }

    // Warm start the constraint (apply the previous impulse at the beginning of the step)
    @Override
    public void warmstart(ConstraintSolverData constraintSolverData) {

        // Get the velocities
        Vector3 v1 = constraintSolverData.linearVelocities[mIndexBody1];
        Vector3 v2 = constraintSolverData.linearVelocities[mIndexBody2];
        Vector3 w1 = constraintSolverData.angularVelocities[mIndexBody1];
        Vector3 w2 = constraintSolverData.angularVelocities[mIndexBody2];

        // Get the inverse mass of the bodies
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda
            Vector3 linearImpulseBody1 = new Vector3(mImpulse).invert();
            Vector3 angularImpulseBody1 = new Vector3(mImpulse).cross(mR1World);

            // Apply the impulse to the body
            v1.add(new Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            w1.add(mI1.multiply(angularImpulseBody1, new Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda
            Vector3 linearImpulseBody2 = mImpulse;
            Vector3 angularImpulseBody2 = new Vector3(new Vector3(mImpulse).cross(mR2World)).invert();

            // Apply the impulse to the body
            v2.add(new Vector3(linearImpulseBody2).multiply(inverseMassBody2));
            w2.add(mI2.multiply(angularImpulseBody2, new Vector3()));
        }
    }

    // Solve the velocity constraint
    @Override
    public void solveVelocityConstraint(ConstraintSolverData constraintSolverData) {

        // Get the velocities
        Vector3 v1 = constraintSolverData.linearVelocities[mIndexBody1];
        Vector3 v2 = constraintSolverData.linearVelocities[mIndexBody2];
        Vector3 w1 = constraintSolverData.angularVelocities[mIndexBody1];
        Vector3 w2 = constraintSolverData.angularVelocities[mIndexBody2];

        // Get the inverse mass of the bodies
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();

        // Compute J*v
        Vector3 Jv = new Vector3(new Vector3(new Vector3(v2).add(new Vector3(w2).cross(mR2World))).subtract(v1)).subtract(new Vector3(w1).cross(mR1World));

        // Compute the Lagrange multiplier lambda
        Vector3 deltaLambda = mInverseMassMatrix.multiply(
                new Vector3(new Vector3(Jv).invert()).subtract(mBiasVector), new Vector3());
        mImpulse.add(deltaLambda);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda
            Vector3 linearImpulseBody1 = new Vector3(deltaLambda).invert();
            Vector3 angularImpulseBody1 = new Vector3(deltaLambda).cross(mR1World);

            // Apply the impulse to the body
            v1.add(new Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            w1.add(mI1.multiply(angularImpulseBody1, new Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda
            Vector3 linearImpulseBody2 = deltaLambda;
            Vector3 angularImpulseBody2 = new Vector3(deltaLambda).cross(mR2World).invert();

            // Apply the impulse to the body
            v2.add(new Vector3(linearImpulseBody2).multiply(inverseMassBody2));
            w2.add(mI2.multiply(angularImpulseBody2, new Vector3()));
        }
    }

    // Solve the position constraint (for position error correction)
    @Override
    public void solvePositionConstraint(ConstraintSolverData constraintSolverData) {

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mPositionCorrectionTechnique != JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL) {
            return;
        }

        // Get the bodies positions and orientations
        Vector3 x1 = constraintSolverData.positions.get(mIndexBody1);
        Vector3 x2 = constraintSolverData.positions.get(mIndexBody2);
        Quaternion q1 = constraintSolverData.orientations.get(mIndexBody1);
        Quaternion q2 = constraintSolverData.orientations.get(mIndexBody2);

        // Get the inverse mass and inverse inertia tensors of the bodies
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();

        // Recompute the inverse inertia tensors
        mI1 = mBody1.getInertiaTensorInverseWorld();
        mI2 = mBody2.getInertiaTensorInverseWorld();

        // Compute the vector from body center to the anchor point in world-space
        q1.multiply(mLocalAnchorPointBody1, mR1World);
        q2.multiply(mLocalAnchorPointBody2, mR2World);

        // Compute the corresponding skew-symmetric matrices
        Matrix3x3 skewSymmetricMatrixU1 = new Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(mR1World);
        Matrix3x3 skewSymmetricMatrixU2 = new Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(mR2World);

        // Recompute the inverse mass matrix K=J^TM^-1J of of the 3 translation constraints
        float inverseMassBodies = 0.0f;
        if (mBody1.isMotionEnabled()) {
            inverseMassBodies += inverseMassBody1;
        }
        if (mBody2.isMotionEnabled()) {
            inverseMassBodies += inverseMassBody2;
        }
        Matrix3x3 massMatrix = new Matrix3x3(inverseMassBodies, 0.0f, 0.0f,
                0.0f, inverseMassBodies, 0.0f,
                0.0f, 0.0f, inverseMassBodies);
        if (mBody1.isMotionEnabled()) {
            massMatrix.add(new Matrix3x3(
                    skewSymmetricMatrixU1).multiply(new Matrix3x3(mI1).multiply(new Matrix3x3(skewSymmetricMatrixU1).transpose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.add(new Matrix3x3(
                    skewSymmetricMatrixU2).multiply(new Matrix3x3(mI2).multiply(new Matrix3x3(skewSymmetricMatrixU2).transpose())));
        }
        mInverseMassMatrix.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrix = new Matrix3x3(massMatrix).inverse();
        }

        // Compute the constraint error (value of the C(x) function)
        Vector3 constraintError = new Vector3(new Vector3(new Vector3(x2).add(mR2World)).subtract(x1)).subtract(mR1World);

        // Compute the Lagrange multiplier lambda
        // TODO : Do not solve the system by computing the inverse each time and multiplying with the
        //        right-hand side vector but instead use a method to directly solve the linear system.
        Vector3 lambda = mInverseMassMatrix.multiply(new Vector3(constraintError).invert(), new Vector3());

        // Apply the impulse to the bodies of the joint (directly update the position/orientation)
        if (mBody1.isMotionEnabled()) {

            // Compute the impulse
            Vector3 linearImpulseBody1 = new Vector3(lambda).invert();
            Vector3 angularImpulseBody1 = new Vector3(lambda).cross(mR1World);

            // Compute the pseudo velocity
            Vector3 v1 = new Vector3(linearImpulseBody1).multiply(inverseMassBody1);
            Vector3 w1 = mI1.multiply(angularImpulseBody1, new Vector3());

            // Update the body position/orientation
            x1.add(v1);
            q1.add(new Quaternion(w1, 0.0f).multiply(q1).multiply(0.5f));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse
            Vector3 linearImpulseBody2 = lambda;
            Vector3 angularImpulseBody2 = new Vector3(lambda).cross(mR2World).invert();

            // Compute the pseudo velocity
            Vector3 v2 = new Vector3(linearImpulseBody2).multiply(inverseMassBody2);
            Vector3 w2 = mI2.multiply(angularImpulseBody2, new Vector3());

            // Update the body position/orientation
            x2.add(v2);
            q2.add(new Quaternion(w2, 0.0f).multiply(q2).multiply(0.5f));
            q2.normalize();
        }
    }

}
