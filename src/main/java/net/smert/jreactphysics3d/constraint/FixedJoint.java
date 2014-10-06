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
 * This class represents a fixed joint that is used to forbid any translation or rotation between two bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class FixedJoint extends Joint {

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

    // Accumulated impulse for the 3 translation constraints
    private Vector3 mImpulseTranslation;

    // Accumulate impulse for the 3 rotation constraints
    private Vector3 mImpulseRotation;

    // Inverse mass matrix K=JM^-1J^-t of the 3 translation constraints (3x3 matrix)
    private Matrix3x3 mInverseMassMatrixTranslation;

    // Inverse mass matrix K=JM^-1J^-t of the 3 rotation constraints (3x3 matrix)
    private Matrix3x3 mInverseMassMatrixRotation;

    // Bias vector for the 3 translation constraints
    private Vector3 mBiasTranslation;

    // Bias vector for the 3 rotation constraints
    private Vector3 mBiasRotation;

    // Inverse of the initial orientation difference between the two bodies
    private Quaternion mInitOrientationDifferenceInv;

    // Constructor
    public FixedJoint(FixedJointInfo jointInfo) {
        super(jointInfo);

        mImpulseTranslation = new Vector3();
        mImpulseRotation = new Vector3();

        // Compute the local-space anchor point for each body
        Transform transform1 = mBody1.getTransform();
        Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = new Transform(transform1).inverse().multiply(jointInfo.anchorPointWorldSpace, new Vector3());
        mLocalAnchorPointBody2 = new Transform(transform2).inverse().multiply(jointInfo.anchorPointWorldSpace, new Vector3());

        // Compute the inverse of the initial orientation difference between the two bodies
        mInitOrientationDifferenceInv = new Quaternion(transform2.getOrientation()).multiply(new Quaternion(transform1.getOrientation()).inverse());
        mInitOrientationDifferenceInv.normalize();
        mInitOrientationDifferenceInv.inverse();
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

        // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
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

        // Compute the inverse mass matrix K^-1 for the 3 translation constraints
        mInverseMassMatrixTranslation.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation = new Matrix3x3(massMatrix).inverse();
        }

        // Compute the bias "b" of the constraint for the 3 translation constraints
        float biasFactor = (BETA / constraintSolverData.timeStep);
        mBiasTranslation.zero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBiasTranslation = new Vector3(
                    new Vector3(new Vector3(new Vector3(x2).add(mR2World)).subtract(x1)).subtract(mR1World)).multiply(biasFactor);
        }

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mInverseMassMatrixRotation.zero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotation.add(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation.add(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation = new Matrix3x3(mInverseMassMatrixRotation).inverse();
        }

        // Compute the bias "b" for the 3 rotation constraints
        mBiasRotation.zero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            Quaternion currentOrientationDifference = new Quaternion(orientationBody2).multiply(new Quaternion(orientationBody1).inverse());
            currentOrientationDifference.normalize();
            Quaternion qError = new Quaternion(currentOrientationDifference).multiply(mInitOrientationDifferenceInv);
            Vector3 qErrorV = new Vector3();
            qError.getVectorV(qErrorV);
            mBiasRotation = qErrorV.multiply(biasFactor * 2.0f);
        }

        // If warm-starting is not enabled
        if (!constraintSolverData.isWarmStartingActive) {

            // Reset the accumulated impulses
            mImpulseTranslation.zero();
            mImpulseRotation.zero();
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

            // Compute the impulse P=J^T * lambda for the 3 translation constraints
            Vector3 linearImpulseBody1 = new Vector3(mImpulseTranslation).invert();
            Vector3 angularImpulseBody1 = new Vector3(mImpulseTranslation).cross(mR1World);

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            angularImpulseBody1.add(new Vector3(mImpulseRotation).invert());

            // Apply the impulse to the body
            v1.add(new Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            w1.add(mI1.multiply(angularImpulseBody1, new Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 translation constraints
            Vector3 linearImpulseBody2 = mImpulseTranslation;
            Vector3 angularImpulseBody2 = new Vector3(mImpulseTranslation).cross(mR2World).invert();

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            angularImpulseBody2.add(mImpulseRotation);

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

        /**
         * --------------- Translation Constraints ---------------
         */
        // Compute J*v for the 3 translation constraints
        Vector3 JvTranslation = new Vector3(
                new Vector3(new Vector3(v2).add(new Vector3(w2).cross(mR2World))).subtract(v1)).subtract(new Vector3(w1).cross(mR1World));

        // Compute the Lagrange multiplier lambda
        Vector3 deltaLambda = mInverseMassMatrixTranslation.multiply(
                new Vector3(new Vector3(JvTranslation).invert()).subtract(mBiasTranslation), new Vector3());
        mImpulseTranslation.add(deltaLambda);

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

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute J*v for the 3 rotation constraints
        Vector3 JvRotation = new Vector3(w2).subtract(w1);

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        Vector3 deltaLambda2 = mInverseMassMatrixRotation.multiply(
                new Vector3(new Vector3(JvRotation).invert()).subtract(mBiasRotation), new Vector3());
        mImpulseRotation.add(deltaLambda2);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody1 = new Vector3(deltaLambda2).invert();

            // Apply the impulse to the body
            w1.add(mI1.multiply(angularImpulseBody1, new Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody2 = deltaLambda2;

            // Apply the impulse to the body
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

        /**
         * --------------- Translation Constraints ---------------
         */
        // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
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
        mInverseMassMatrixTranslation.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation = new Matrix3x3(massMatrix).inverse();
        }

        // Compute position error for the 3 translation constraints
        Vector3 errorTranslation = new Vector3(
                new Vector3(new Vector3(x2).add(mR2World)).subtract(x1)).subtract(mR1World);

        // Compute the Lagrange multiplier lambda
        Vector3 lambdaTranslation = mInverseMassMatrixTranslation.multiply(
                new Vector3(errorTranslation).invert(), new Vector3());

        // Apply the impulse to the bodies of the joint
        if (mBody1.isMotionEnabled()) {

            // Compute the impulse
            Vector3 linearImpulseBody1 = new Vector3(lambdaTranslation).invert();
            Vector3 angularImpulseBody1 = new Vector3(lambdaTranslation).cross(mR1World);

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
            Vector3 linearImpulseBody2 = lambdaTranslation;
            Vector3 angularImpulseBody2 = new Vector3(lambdaTranslation).cross(mR2World).invert();

            // Compute the pseudo velocity
            Vector3 v2 = new Vector3(linearImpulseBody2).multiply(inverseMassBody2);
            Vector3 w2 = mI2.multiply(angularImpulseBody2, new Vector3());

            // Update the body position/orientation
            x2.add(v2);
            q2.add(new Quaternion(w2, 0.0f).multiply(q2).multiply(0.5f));
            q2.normalize();
        }

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mInverseMassMatrixRotation.zero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotation.add(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation.add(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation = new Matrix3x3(mInverseMassMatrixRotation).inverse();
        }

        // Compute the position error for the 3 rotation constraints
        Quaternion currentOrientationDifference = new Quaternion(q2).multiply(new Quaternion(q1).inverse());
        currentOrientationDifference.normalize();
        Quaternion qError = new Quaternion(currentOrientationDifference).multiply(mInitOrientationDifferenceInv);
        Vector3 qErrorV = new Vector3();
        qError.getVectorV(qErrorV);
        Vector3 errorRotation = qErrorV.multiply(2.0f);

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        Vector3 lambdaRotation = mInverseMassMatrixRotation.multiply(
                new Vector3(errorRotation).invert(), new Vector3());

        // Apply the impulse to the bodies of the joint
        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody1 = new Vector3(lambdaRotation).invert();

            // Compute the pseudo velocity
            Vector3 w1 = mI1.multiply(angularImpulseBody1, new Vector3());

            // Update the body position/orientation
            q1.add(new Quaternion(w1, 0.0f).multiply(q1).multiply(0.5f));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse
            Vector3 angularImpulseBody2 = lambdaRotation;

            // Compute the pseudo velocity
            Vector3 w2 = mI2.multiply(angularImpulseBody2, new Vector3());

            // Update the body position/orientation
            q2.add(new Quaternion(w2, 0.0f).multiply(q2).multiply(0.5f));
            q2.normalize();
        }
    }

}
