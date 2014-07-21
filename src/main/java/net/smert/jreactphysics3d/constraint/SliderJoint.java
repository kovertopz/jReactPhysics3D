package net.smert.jreactphysics3d.constraint;

import net.smert.jreactphysics3d.configuration.JointsPositionCorrectionTechnique;
import net.smert.jreactphysics3d.engine.ConstraintSolverData;
import net.smert.jreactphysics3d.mathematics.Matrix2x2;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector2;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a slider joint. This joint has a one degree of freedom. It only allows relative translation of
 * the bodies along a single direction and no rotation.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class SliderJoint extends Joint {

    // Beta value for the position correction bias factor
    private static final float BETA = 0.2f;

    /// Anchor point of body 1 (in local-space coordinates of body 1)
    private Vector3 mLocalAnchorPointBody1;

    /// Anchor point of body 2 (in local-space coordinates of body 2)
    private Vector3 mLocalAnchorPointBody2;

    /// Slider axis (in local-space coordinates of body 1)
    private Vector3 mSliderAxisBody1;

    /// Inertia tensor of body 1 (in world-space coordinates)
    private Matrix3x3 mI1;

    /// Inertia tensor of body 2 (in world-space coordinates)
    private Matrix3x3 mI2;

    /// Inverse of the initial orientation difference between the two bodies
    private Quaternion mInitOrientationDifferenceInv;

    /// First vector orthogonal to the slider axis local-space of body 1
    private Vector3 mN1;

    /// Second vector orthogonal to the slider axis and mN1 in local-space of body 1
    private Vector3 mN2;

    /// Vector r1 in world-space coordinates
    private Vector3 mR1;

    /// Vector r2 in world-space coordinates
    private Vector3 mR2;

    /// Cross product of r2 and n1
    private Vector3 mR2CrossN1;

    /// Cross product of r2 and n2
    private Vector3 mR2CrossN2;

    /// Cross product of r2 and the slider axis
    private Vector3 mR2CrossSliderAxis;

    /// Cross product of vector (r1 + u) and n1
    private Vector3 mR1PlusUCrossN1;

    /// Cross product of vector (r1 + u) and n2
    private Vector3 mR1PlusUCrossN2;

    /// Cross product of vector (r1 + u) and the slider axis
    private Vector3 mR1PlusUCrossSliderAxis;

    /// Bias of the 2 translation constraints
    private Vector2 mBTranslation;

    /// Bias of the 3 rotation constraints
    private Vector3 mBRotation;

    /// Bias of the lower limit constraint
    private float mBLowerLimit;

    /// Bias of the upper limit constraint
    private float mBUpperLimit;

    /// Inverse of mass matrix K=JM^-1J^t for the translation constraint (2x2 matrix)
    private Matrix2x2 mInverseMassMatrixTranslationConstraint;

    /// Inverse of mass matrix K=JM^-1J^t for the rotation constraint (3x3 matrix)
    private Matrix3x3 mInverseMassMatrixRotationConstraint;

    /// Inverse of mass matrix K=JM^-1J^t for the upper and lower limit constraints (1x1 matrix)
    private float mInverseMassMatrixLimit;

    /// Inverse of mass matrix K=JM^-1J^t for the motor
    private float mInverseMassMatrixMotor;

    /// Accumulated impulse for the 2 translation constraints
    private Vector2 mImpulseTranslation;

    /// Accumulated impulse for the 3 rotation constraints
    private Vector3 mImpulseRotation;

    /// Accumulated impulse for the lower limit constraint
    private float mImpulseLowerLimit;

    /// Accumulated impulse for the upper limit constraint
    private float mImpulseUpperLimit;

    /// Accumulated impulse for the motor
    private float mImpulseMotor;

    /// True if the slider limits are enabled
    private boolean mIsLimitEnabled;

    /// True if the motor of the joint in enabled
    private boolean mIsMotorEnabled;

    /// Slider axis in world-space coordinates
    private Vector3 mSliderAxisWorld;

    /// Lower limit (minimum translation distance)
    private float mLowerLimit;

    /// Upper limit (maximum translation distance)
    private float mUpperLimit;

    /// True if the lower limit is violated
    private boolean mIsLowerLimitViolated;

    /// True if the upper limit is violated
    private boolean mIsUpperLimitViolated;

    /// Motor speed
    private float mMotorSpeed;

    /// Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
    private float mMaxMotorForce;

    // -------------------- Methods -------------------- //
    /// Private copy-constructor
    private SliderJoint(SliderJoint constraint) {
        super(constraint);
    }

    /// Private assignment operator
    private SliderJoint operatorEqual(SliderJoint constraint) {
        return this;
    }

    // Reset the limits
    protected void resetLimits() {

        // Reset the accumulated impulses for the limits
        mImpulseLowerLimit = 0.0f;
        mImpulseUpperLimit = 0.0f;

        // Wake up the two bodies of the joint
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    // Return the number of bytes used by the joint
    @Override
    protected int getSizeInBytes() {
        return 4;
    }

    // Initialize before solving the constraint
    @Override
    protected void initBeforeSolve(ConstraintSolverData constraintSolverData) {

        // Initialize the bodies index in the veloc ity array
        mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody1).second;
        mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody2).second;

        // Get the bodies positions and orientations
        Vector3 x1 = mBody1.getTransform().getPosition();
        Vector3 x2 = mBody2.getTransform().getPosition();
        Quaternion orientationBody1 = mBody1.getTransform().getOrientation();
        Quaternion orientationBody2 = mBody2.getTransform().getOrientation();

        // Get the inertia tensor of bodies
        mI1 = mBody1.getInertiaTensorInverseWorld();
        mI2 = mBody2.getInertiaTensorInverseWorld();

        // Vector from body center to the anchor point
        mR1 = orientationBody1 * mLocalAnchorPointBody1;
        mR2 = orientationBody2 * mLocalAnchorPointBody2;

        // Compute the vector u (difference between anchor points)
        Vector3 u = x2 + mR2 - x1 - mR1;

        // Compute the two orthogonal vectors to the slider axis in world-space
        mSliderAxisWorld = orientationBody1 * mSliderAxisBody1;
        mSliderAxisWorld.normalize();
        mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
        mN2 = mSliderAxisWorld.cross(mN1);

        // Check if the limit constraints are violated or not
        float uDotSliderAxis = u.dot(mSliderAxisWorld);
        float lowerLimitError = uDotSliderAxis - mLowerLimit;
        float upperLimitError = mUpperLimit - uDotSliderAxis;
        boolean oldIsLowerLimitViolated = mIsLowerLimitViolated;
        mIsLowerLimitViolated = lowerLimitError <= 0.0f;
        if (mIsLowerLimitViolated != oldIsLowerLimitViolated) {
            mImpulseLowerLimit = 0.0f;
        }
        boolean oldIsUpperLimitViolated = mIsUpperLimitViolated;
        mIsUpperLimitViolated = upperLimitError <= 0.0f;
        if (mIsUpperLimitViolated != oldIsUpperLimitViolated) {
            mImpulseUpperLimit = 0.0f;
        }

        // Compute the cross products used in the Jacobians
        mR2CrossN1 = mR2.cross(mN1);
        mR2CrossN2 = mR2.cross(mN2);
        mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
        Vector3 r1PlusU = mR1 + u;
        mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
        mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
        mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
        // constraints (2x2 matrix)
        float sumInverseMass = 0.0f;
        Vector3 I1R1PlusUCrossN1 = new Vector3(0.0f, 0.0f, 0.0f);
        Vector3 I1R1PlusUCrossN2 = new Vector3(0.0f, 0.0f, 0.0f);
        Vector3 I2R2CrossN1 = new Vector3(0.0f, 0.0f, 0.0f);
        Vector3 I2R2CrossN2 = new Vector3(0.0f, 0.0f, 0.0f);
        if (mBody1.isMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
            I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
        }
        if (mBody2.isMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1 = mI2 * mR2CrossN1;
            I2R2CrossN2 = mI2 * mR2CrossN2;
        }
        float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
        float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
        float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
        float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
        Matrix2x2 matrixKTranslation = new Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixTranslationConstraint.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
        }

        // Compute the bias "b" of the translation constraint
        mBTranslation.setToZero();
        float biasFactor = (BETA / constraintSolverData.timeStep);
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation.x = u.dot(mN1);
            mBTranslation.y = u.dot(mN2);
            mBTranslation *= biasFactor;
        }

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mInverseMassMatrixRotationConstraint.setToZero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint += mI1;
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint += mI2;
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
        }

        // Compute the bias "b" of the rotation constraint
        mBRotation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            Quaternion currentOrientationDifference = orientationBody2 * orientationBody1.getInverse();
            currentOrientationDifference.normalize();
            Quaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
            mBRotation = biasFactor * 2.0f * qError.getVectorV();
        }

        // If the limits are enabled
        if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated)) {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            mInverseMassMatrixLimit = 0.0f;
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixLimit += mBody1.getMassInverse() + mR1PlusUCrossSliderAxis.dot(mI1 * mR1PlusUCrossSliderAxis);
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixLimit += mBody2.getMassInverse() + mR2CrossSliderAxis.dot(mI2 * mR2CrossSliderAxis);
            }
            mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0f) ? 1.0f / mInverseMassMatrixLimit : 0.0f;

            // Compute the bias "b" of the lower limit constraint
            mBLowerLimit = 0.0f;
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBLowerLimit = biasFactor * lowerLimitError;
            }

            // Compute the bias "b" of the upper limit constraint
            mBUpperLimit = 0.0f;
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBUpperLimit = biasFactor * upperLimitError;
            }
        }

        // If the motor is enabled
        if (mIsMotorEnabled) {

            // Compute the inverse of mass matrix K=JM^-1J^t for the motor (1x1 matrix)
            mInverseMassMatrixMotor = 0.0f;
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixMotor += mBody1.getMassInverse();
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixMotor += mBody2.getMassInverse();
            }
            mInverseMassMatrixMotor = (mInverseMassMatrixMotor > 0.0f) ? 1.0f / mInverseMassMatrixMotor : 0.0f;
        }

        // If warm-starting is not enabled
        if (!constraintSolverData.isWarmStartingActive) {

            // Reset all the accumulated impulses
            mImpulseTranslation.setToZero();
            mImpulseRotation.setToZero();
            mImpulseLowerLimit = 0.0f;
            mImpulseUpperLimit = 0.0f;
            mImpulseMotor = 0.0f;
        }
    }

    // Warm start the constraint (apply the previous impulse at the beginning of the step)
    @Override
    protected void warmstart(ConstraintSolverData constraintSolverData) {

        // Get the velocities
        Vector3 v1 = constraintSolverData.linearVelocities[mIndexBody1];
        Vector3 v2 = constraintSolverData.linearVelocities[mIndexBody2];
        Vector3 w1 = constraintSolverData.angularVelocities[mIndexBody1];
        Vector3 w2 = constraintSolverData.angularVelocities[mIndexBody2];

        // Get the inverse mass and inverse inertia tensors of the bodies
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
        float impulseLimits = mImpulseUpperLimit - mImpulseLowerLimit;
        Vector3 linearImpulseLimits = impulseLimits * mSliderAxisWorld;

        // Compute the impulse P=J^T * lambda for the motor constraint
        Vector3 impulseMotor = mImpulseMotor * mSliderAxisWorld;

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody1 = -mN1 * mImpulseTranslation.x - mN2 * mImpulseTranslation.y;
            Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * mImpulseTranslation.x - mR1PlusUCrossN2 * mImpulseTranslation.y;

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            angularImpulseBody1 += -mImpulseRotation;

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            linearImpulseBody1 += linearImpulseLimits;
            angularImpulseBody1 += impulseLimits * mR1PlusUCrossSliderAxis;

            // Compute the impulse P=J^T * lambda for the motor constraint
            linearImpulseBody1 += impulseMotor;

            // Apply the impulse to the body
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody2 = mN1 * mImpulseTranslation.x + mN2 * mImpulseTranslation.y;
            Vector3 angularImpulseBody2 = mR2CrossN1 * mImpulseTranslation.x
                    + mR2CrossN2 * mImpulseTranslation.y;

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            angularImpulseBody2 += mImpulseRotation;

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            linearImpulseBody2 += -linearImpulseLimits;
            angularImpulseBody2 += -impulseLimits * mR2CrossSliderAxis;

            // Compute the impulse P=J^T * lambda for the motor constraint
            linearImpulseBody2 += -impulseMotor;

            // Apply the impulse to the body
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;
        }
    }

    // Solve the velocity constraint
    @Override
    protected void solveVelocityConstraint(ConstraintSolverData constraintSolverData) {

        // Get the velocities
        Vector3 v1 = constraintSolverData.linearVelocities[mIndexBody1];
        Vector3 v2 = constraintSolverData.linearVelocities[mIndexBody2];
        Vector3 w1 = constraintSolverData.angularVelocities[mIndexBody1];
        Vector3 w2 = constraintSolverData.angularVelocities[mIndexBody2];

        // Get the inverse mass and inverse inertia tensors of the bodies
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();

        /**
         * --------------- Translation Constraints ---------------
         */
        // Compute J*v for the 2 translation constraints
        float el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) + mN1.dot(v2) + w2.dot(mR2CrossN1);
        float el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) + mN2.dot(v2) + w2.dot(mR2CrossN2);
        Vector2 JvTranslation = new Vector2(el1, el2);

        // Compute the Lagrange multiplier lambda for the 2 translation constraints
        Vector2 deltaLambda = mInverseMassMatrixTranslationConstraint * (-JvTranslation - mBTranslation);
        mImpulseTranslation += deltaLambda;

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody1 = -mN1 * deltaLambda.x - mN2 * deltaLambda.y;
            Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * deltaLambda.x - mR1PlusUCrossN2 * deltaLambda.y;

            // Apply the impulse to the body
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody2 = mN1 * deltaLambda.x + mN2 * deltaLambda.y;
            Vector3 angularImpulseBody2 = mR2CrossN1 * deltaLambda.x + mR2CrossN2 * deltaLambda.y;

            // Apply the impulse to the body
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;
        }

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute J*v for the 3 rotation constraints
        Vector3 JvRotation = w2 - w1;

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        Vector3 deltaLambda2 = mInverseMassMatrixRotationConstraint * (-JvRotation - mBRotation);
        mImpulseRotation += deltaLambda2;

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody1 = -deltaLambda2;

            // Apply the impulse to the body
            w1 += mI1 * angularImpulseBody1;
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody2 = deltaLambda2;

            // Apply the impulse to the body
            w2 += mI2 * angularImpulseBody2;
        }

        /**
         * --------------- Limits Constraints ---------------
         */
        if (mIsLimitEnabled) {

            // If the lower limit is violated
            if (mIsLowerLimitViolated) {

                // Compute J*v for the lower limit constraint
                float JvLowerLimit = mSliderAxisWorld.dot(v2) + mR2CrossSliderAxis.dot(w2)
                        - mSliderAxisWorld.dot(v1) - mR1PlusUCrossSliderAxis.dot(w1);

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                float deltaLambdaLower = mInverseMassMatrixLimit * (-JvLowerLimit - mBLowerLimit);
                float lambdaTemp = mImpulseLowerLimit;
                mImpulseLowerLimit = (float) Math.max(mImpulseLowerLimit + deltaLambdaLower, 0.0f);
                deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    Vector3 linearImpulseBody1 = -deltaLambdaLower * mSliderAxisWorld;
                    Vector3 angularImpulseBody1 = -deltaLambdaLower * mR1PlusUCrossSliderAxis;

                    // Apply the impulse to the body
                    v1 += inverseMassBody1 * linearImpulseBody1;
                    w1 += mI1 * angularImpulseBody1;
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    Vector3 linearImpulseBody2 = deltaLambdaLower * mSliderAxisWorld;
                    Vector3 angularImpulseBody2 = deltaLambdaLower * mR2CrossSliderAxis;

                    // Apply the impulse to the body
                    v2 += inverseMassBody2 * linearImpulseBody2;
                    w2 += mI2 * angularImpulseBody2;
                }
            }

            // If the upper limit is violated
            if (mIsUpperLimitViolated) {

                // Compute J*v for the upper limit constraint
                float JvUpperLimit = mSliderAxisWorld.dot(v1) + mR1PlusUCrossSliderAxis.dot(w1)
                        - mSliderAxisWorld.dot(v2) - mR2CrossSliderAxis.dot(w2);

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                float deltaLambdaUpper = mInverseMassMatrixLimit * (-JvUpperLimit - mBUpperLimit);
                float lambdaTemp = mImpulseUpperLimit;
                mImpulseUpperLimit = Math.max(mImpulseUpperLimit + deltaLambdaUpper, 0.0f);
                deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    Vector3 linearImpulseBody1 = deltaLambdaUpper * mSliderAxisWorld;
                    Vector3 angularImpulseBody1 = deltaLambdaUpper * mR1PlusUCrossSliderAxis;

                    // Apply the impulse to the body
                    v1 += inverseMassBody1 * linearImpulseBody1;
                    w1 += mI1 * angularImpulseBody1;
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    Vector3 linearImpulseBody2 = -deltaLambdaUpper * mSliderAxisWorld;
                    Vector3 angularImpulseBody2 = -deltaLambdaUpper * mR2CrossSliderAxis;

                    // Apply the impulse to the body
                    v2 += inverseMassBody2 * linearImpulseBody2;
                    w2 += mI2 * angularImpulseBody2;
                }
            }
        }

        /**
         * --------------- Motor ---------------
         */
        if (mIsMotorEnabled) {

            // Compute J*v for the motor
            float JvMotor = mSliderAxisWorld.dot(v1) - mSliderAxisWorld.dot(v2);

            // Compute the Lagrange multiplier lambda for the motor
            float maxMotorImpulse = mMaxMotorForce * constraintSolverData.timeStep;
            float deltaLambdaMotor = mInverseMassMatrixMotor * (-JvMotor - mMotorSpeed);
            float lambdaTemp = mImpulseMotor;
            mImpulseMotor = Math.clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mImpulseMotor - lambdaTemp;

            if (mBody1.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                Vector3 linearImpulseBody1 = deltaLambdaMotor * mSliderAxisWorld;

                // Apply the impulse to the body
                v1 += inverseMassBody1 * linearImpulseBody1;
            }
            if (mBody2.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                Vector3 linearImpulseBody2 = -deltaLambdaMotor * mSliderAxisWorld;

                // Apply the impulse to the body
                v2 += inverseMassBody2 * linearImpulseBody2;
            }
        }
    }

    // Solve the position constraint (for position error correction)
    @Override
    protected void solvePositionConstraint(ConstraintSolverData constraintSolverData) {

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mPositionCorrectionTechnique != JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL) {
            return;
        }

        // Get the bodies positions and orientations
        Vector3 x1 = constraintSolverData.positions[mIndexBody1];
        Vector3 x2 = constraintSolverData.positions[mIndexBody2];
        Quaternion q1 = constraintSolverData.orientations[mIndexBody1];
        Quaternion q2 = constraintSolverData.orientations[mIndexBody2];

        // Get the inverse mass and inverse inertia tensors of the bodies
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();

        // Recompute the inertia tensor of bodies
        mI1 = mBody1.getInertiaTensorInverseWorld();
        mI2 = mBody2.getInertiaTensorInverseWorld();

        // Vector from body center to the anchor point
        mR1 = q1 * mLocalAnchorPointBody1;
        mR2 = q2 * mLocalAnchorPointBody2;

        // Compute the vector u (difference between anchor points)
        Vector3 u = x2 + mR2 - x1 - mR1;

        // Compute the two orthogonal vectors to the slider axis in world-space
        mSliderAxisWorld = q1 * mSliderAxisBody1;
        mSliderAxisWorld.normalize();
        mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
        mN2 = mSliderAxisWorld.cross(mN1);

        // Check if the limit constraints are violated or not
        float uDotSliderAxis = u.dot(mSliderAxisWorld);
        float lowerLimitError = uDotSliderAxis - mLowerLimit;
        float upperLimitError = mUpperLimit - uDotSliderAxis;
        mIsLowerLimitViolated = lowerLimitError <= 0.0f;
        mIsUpperLimitViolated = upperLimitError <= 0.0f;

        // Compute the cross products used in the Jacobians
        mR2CrossN1 = mR2.cross(mN1);
        mR2CrossN2 = mR2.cross(mN2);
        mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
        Vector3 r1PlusU = mR1 + u;
        mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
        mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
        mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

        /**
         * --------------- Translation Constraints ---------------
         */
        // Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
        // constraints (2x2 matrix)
        float sumInverseMass = 0.0f;
        Vector3 I1R1PlusUCrossN1 = new Vector3(0.0f, 0.0f, 0.0f);
        Vector3 I1R1PlusUCrossN2 = new Vector3(0.0f, 0.0f, 0.0f);
        Vector3 I2R2CrossN1 = new Vector3(0.0f, 0.0f, 0.0f);
        Vector3 I2R2CrossN2 = new Vector3(0.0f, 0.0f, 0.0f);
        if (mBody1.isMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
            I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
        }
        if (mBody2.isMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1 = mI2 * mR2CrossN1;
            I2R2CrossN2 = mI2 * mR2CrossN2;
        }
        float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
        float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
        float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
        float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
        Matrix2x2 matrixKTranslation = new Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixTranslationConstraint.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
        }

        // Compute the position error for the 2 translation constraints
        Vector2 translationError = new Vector2(u.dot(mN1), u.dot(mN2));

        // Compute the Lagrange multiplier lambda for the 2 translation constraints
        Vector2 lambdaTranslation = mInverseMassMatrixTranslationConstraint * (-translationError);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody1 = -mN1 * lambdaTranslation.x - mN2 * lambdaTranslation.y;
            Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * lambdaTranslation.x - mR1PlusUCrossN2 * lambdaTranslation.y;

            // Apply the impulse to the body
            Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
            Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation
            x1 += v1;
            q1 += new Quaternion(0.0f, w1) * q1 * 0.5f;
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody2 = mN1 * lambdaTranslation.x + mN2 * lambdaTranslation.y;
            Vector3 angularImpulseBody2 = mR2CrossN1 * lambdaTranslation.x + mR2CrossN2 * lambdaTranslation.y;

            // Apply the impulse to the body
            Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
            Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation
            x2 += v2;
            q2 += new Quaternion(0.0f, w2) * q2 * 0.5f;
            q2.normalize();
        }

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mInverseMassMatrixRotationConstraint.setToZero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint += mI1;
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint += mI2;
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
        }

        // Compute the position error for the 3 rotation constraints
        Quaternion currentOrientationDifference = q2 * q1.getInverse();
        currentOrientationDifference.normalize();
        Quaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
        Vector3 errorRotation = 2.0f * qError.getVectorV();

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        Vector3 lambdaRotation = mInverseMassMatrixRotationConstraint * (-errorRotation);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody1 = -lambdaRotation;

            // Apply the impulse to the body
            Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation
            q1 += new Quaternion(0.0f, w1) * q1 * 0.5f;
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody2 = lambdaRotation;

            // Apply the impulse to the body
            Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation
            q2 += new Quaternion(0.0f, w2) * q2 * 0.5f;
            q2.normalize();
        }

        /**
         * --------------- Limits Constraints ---------------
         */
        if (mIsLimitEnabled) {

            if (mIsLowerLimitViolated || mIsUpperLimitViolated) {

                // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
                mInverseMassMatrixLimit = 0.0f;
                if (mBody1.isMotionEnabled()) {
                    mInverseMassMatrixLimit += mBody1.getMassInverse() + mR1PlusUCrossSliderAxis.dot(mI1 * mR1PlusUCrossSliderAxis);
                }
                if (mBody2.isMotionEnabled()) {
                    mInverseMassMatrixLimit += mBody2.getMassInverse() + mR2CrossSliderAxis.dot(mI2 * mR2CrossSliderAxis);
                }
                mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0f) ? 1.0f / mInverseMassMatrixLimit : 0.0f;
            }

            // If the lower limit is violated
            if (mIsLowerLimitViolated) {

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                float lambdaLowerLimit = mInverseMassMatrixLimit * (-lowerLimitError);

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    Vector3 linearImpulseBody1 = -lambdaLowerLimit * mSliderAxisWorld;
                    Vector3 angularImpulseBody1 = -lambdaLowerLimit * mR1PlusUCrossSliderAxis;

                    // Apply the impulse to the body
                    Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
                    Vector3 w1 = mI1 * angularImpulseBody1;

                    // Update the body position/orientation
                    x1 += v1;
                    q1 += new Quaternion(0.0f, w1) * q1 * 0.5f;
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    Vector3 linearImpulseBody2 = lambdaLowerLimit * mSliderAxisWorld;
                    Vector3 angularImpulseBody2 = lambdaLowerLimit * mR2CrossSliderAxis;

                    // Apply the impulse to the body
                    Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
                    Vector3 w2 = mI2 * angularImpulseBody2;

                    // Update the body position/orientation
                    x2 += v2;
                    q2 += new Quaternion(0.0f, w2) * q2 * 0.5f;
                    q2.normalize();
                }
            }

            // If the upper limit is violated
            if (mIsUpperLimitViolated) {

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                float lambdaUpperLimit = mInverseMassMatrixLimit * (-upperLimitError);

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    Vector3 linearImpulseBody1 = lambdaUpperLimit * mSliderAxisWorld;
                    Vector3 angularImpulseBody1 = lambdaUpperLimit * mR1PlusUCrossSliderAxis;

                    // Apply the impulse to the body
                    Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
                    Vector3 w1 = mI1 * angularImpulseBody1;

                    // Update the body position/orientation
                    x1 += v1;
                    q1 += new Quaternion(0.0f, w1) * q1 * 0.5f;
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    Vector3 linearImpulseBody2 = -lambdaUpperLimit * mSliderAxisWorld;
                    Vector3 angularImpulseBody2 = -lambdaUpperLimit * mR2CrossSliderAxis;

                    // Apply the impulse to the body
                    Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
                    Vector3 w2 = mI2 * angularImpulseBody2;

                    // Update the body position/orientation
                    x2 += v2;
                    q2 += new Quaternion(0.0f, w2) * q2 * 0.5f;
                    q2.normalize();
                }
            }
        }
    }

    // Constructor
    public SliderJoint(SliderJointInfo jointInfo) {
        super(jointInfo);

        mImpulseTranslation = new Vector2(0.0f, 0.0f);
        mImpulseRotation = new Vector3(0.0f, 0.0f, 0.0f);
        mImpulseLowerLimit = 0.0f;
        mImpulseUpperLimit = 0.0f;
        mImpulseMotor = 0.0f;
        mIsLimitEnabled = jointInfo.isLimitEnabled;
        mIsMotorEnabled = jointInfo.isMotorEnabled;
        mLowerLimit = jointInfo.minTranslationLimit;
        mUpperLimit = jointInfo.maxTranslationLimit;
        mIsLowerLimitViolated = false;
        mIsUpperLimitViolated = false;
        mMotorSpeed = jointInfo.motorSpeed;
        mMaxMotorForce = jointInfo.maxMotorForce;

        assert (mUpperLimit >= 0.0f);
        assert (mLowerLimit <= 0.0f);
        assert (mMaxMotorForce >= 0.0f);

        // Compute the local-space anchor point for each body
        Transform transform1 = mBody1.getTransform();
        Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = transform1.getInverse() * jointInfo.anchorPointWorldSpace;
        mLocalAnchorPointBody2 = transform2.getInverse() * jointInfo.anchorPointWorldSpace;

        // Compute the inverse of the initial orientation difference between the two bodies
        mInitOrientationDifferenceInv = transform2.getOrientation() * transform1.getOrientation().getInverse();
        mInitOrientationDifferenceInv.normalize();
        mInitOrientationDifferenceInv.inverse();

        // Compute the slider axis in local-space of body 1
        mSliderAxisBody1 = mBody1.getTransform().getOrientation().getInverse() * jointInfo.sliderAxisWorldSpace;
        mSliderAxisBody1.normalize();
    }

    // Return true if the limits or the joint are enabled
    public boolean isLimitEnabled() {
        return mIsLimitEnabled;
    }

    // Return true if the motor of the joint is enabled
    public boolean isMotorEnabled() {
        return mIsMotorEnabled;
    }

    // Return the minimum translation limit
    public float getMinTranslationLimit() {
        return mLowerLimit;
    }

    // Return the maximum translation limit
    public float getMaxTranslationLimit() {
        return mUpperLimit;
    }

    // Return the motor speed
    public float getMotorSpeed() {
        return mMotorSpeed;
    }

    // Return the maximum motor force
    public float getMaxMotorForce() {
        return mMaxMotorForce;
    }

    // Return the intensity of the current force applied for the joint motor
    public float getMotorForce(float timeStep) {
        return mImpulseMotor / timeStep;
    }

    // Enable/Disable the limits of the joint
    public void enableLimit(boolean isLimitEnabled) {

        if (isLimitEnabled != mIsLimitEnabled) {

            mIsLimitEnabled = isLimitEnabled;

            // Reset the limits
            resetLimits();
        }
    }

    // Enable/Disable the motor of the joint
    public void enableMotor(boolean isMotorEnabled) {

        mIsMotorEnabled = isMotorEnabled;
        mImpulseMotor = 0.0f;

        // Wake up the two bodies of the joint
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    // Return the current translation value of the joint
    public float getTranslation() {

        // Get the bodies positions and orientations
        Vector3 x1 = mBody1.getTransform().getPosition();
        Vector3 x2 = mBody2.getTransform().getPosition();
        Quaternion q1 = mBody1.getTransform().getOrientation();
        Quaternion q2 = mBody2.getTransform().getOrientation();

        // Compute the two anchor points in world-space coordinates
        Vector3 anchorBody1 = x1 + q1 * mLocalAnchorPointBody1;
        Vector3 anchorBody2 = x2 + q2 * mLocalAnchorPointBody2;

        // Compute the vector u (difference between anchor points)
        Vector3 u = anchorBody2 - anchorBody1;

        // Compute the slider axis in world-space
        Vector3 sliderAxisWorld = q1 * mSliderAxisBody1;
        sliderAxisWorld.normalize();

        // Compute and return the translation value
        return u.dot(sliderAxisWorld);
    }

    // Set the minimum translation limit
    public void setMinTranslationLimit(float lowerLimit) {

        assert (lowerLimit <= mUpperLimit);

        if (lowerLimit != mLowerLimit) {

            mLowerLimit = lowerLimit;

            // Reset the limits
            resetLimits();
        }
    }

    // Set the maximum translation limit
    public void setMaxTranslationLimit(float upperLimit) {

        assert (mLowerLimit <= upperLimit);

        if (upperLimit != mUpperLimit) {

            mUpperLimit = upperLimit;

            // Reset the limits
            resetLimits();
        }
    }

    // Set the motor speed
    public void setMotorSpeed(float motorSpeed) {

        if (motorSpeed != mMotorSpeed) {

            mMotorSpeed = motorSpeed;

            // Wake up the two bodies of the joint
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

    // Set the maximum motor force
    public void setMaxMotorForce(float maxMotorForce) {

        if (maxMotorForce != mMaxMotorForce) {

            assert (mMaxMotorForce >= 0.0f);
            mMaxMotorForce = maxMotorForce;

            // Wake up the two bodies of the joint
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

}
