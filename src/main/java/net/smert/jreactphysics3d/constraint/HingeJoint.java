package net.smert.jreactphysics3d.constraint;

import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.configuration.JointsPositionCorrectionTechnique;
import net.smert.jreactphysics3d.engine.ConstraintSolverData;
import net.smert.jreactphysics3d.mathematics.Mathematics;
import net.smert.jreactphysics3d.mathematics.Matrix2x2;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector2;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a hinge joint that allows arbitrary rotation between two bodies around a single axis. This
 * joint has one degree of freedom. It can be useful to simulate doors or pendulumns.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class HingeJoint extends Joint {

    // Beta value for the bias factor of position correction
    private static final float BETA = 0.2f;

    // Anchor point of body 1 (in local-space coordinates of body 1)
    private Vector3 mLocalAnchorPointBody1;

    // Anchor point of body 2 (in local-space coordinates of body 2)
    private Vector3 mLocalAnchorPointBody2;

    // Hinge rotation axis (in local-space coordinates of body 1)
    private Vector3 mHingeLocalAxisBody1;

    // Hinge rotation axis (in local-space coordiantes of body 2)
    private Vector3 mHingeLocalAxisBody2;

    // Inertia tensor of body 1 (in world-space coordinates)
    private Matrix3x3 mI1;

    // Inertia tensor of body 2 (in world-space coordinates)
    private Matrix3x3 mI2;

    // Hinge rotation axis (in world-space coordinates) computed from body 1
    private Vector3 mA1;

    // Vector from center of body 2 to anchor point in world-space
    private Vector3 mR1World;

    // Vector from center of body 2 to anchor point in world-space
    private Vector3 mR2World;

    // Cross product of vector b2 and a1
    private Vector3 mB2CrossA1;

    // Cross product of vector c2 and a1;
    private Vector3 mC2CrossA1;

    // Impulse for the 3 translation constraints
    private Vector3 mImpulseTranslation;

    // Impulse for the 2 rotation constraints
    private Vector2 mImpulseRotation;

    // Accumulated impulse for the lower limit constraint
    private float mImpulseLowerLimit;

    // Accumulated impulse for the upper limit constraint
    private float mImpulseUpperLimit;

    // Accumulated impulse for the motor constraint;
    private float mImpulseMotor;

    // Inverse mass matrix K=JM^-1J^t for the 3 translation constraints
    private Matrix3x3 mInverseMassMatrixTranslation;

    // Inverse mass matrix K=JM^-1J^t for the 2 rotation constraints
    private Matrix2x2 mInverseMassMatrixRotation;

    // Inverse of mass matrix K=JM^-1J^t for the limits and motor constraints (1x1 matrix)
    private float mInverseMassMatrixLimitMotor;

    // Inverse of mass matrix K=JM^-1J^t for the motor
    private float mInverseMassMatrixMotor;

    // Bias vector for the error correction for the translation constraints
    private Vector3 mBTranslation;

    // Bias vector for the error correction for the rotation constraints
    private Vector2 mBRotation;

    // Bias of the lower limit constraint
    private float mBLowerLimit;

    // Bias of the upper limit constraint
    private float mBUpperLimit;

    // Inverse of the initial orientation difference between the bodies
    private Quaternion mInitOrientationDifferenceInv;

    // True if the joint limits are enabled
    private boolean mIsLimitEnabled;

    // True if the motor of the joint in enabled
    private boolean mIsMotorEnabled;

    // Lower limit (minimum allowed rotation angle in radi)
    private float mLowerLimit;

    // Upper limit (maximum translation distance)
    private float mUpperLimit;

    // True if the lower limit is violated
    private boolean mIsLowerLimitViolated;

    // True if the upper limit is violated
    private boolean mIsUpperLimitViolated;

    // Motor speed
    private float mMotorSpeed;

    // Maximum motor torque (in Newtons) that can be applied to reach to desired motor speed
    private float mMaxMotorTorque;

    // Reset the limits
    private void resetLimits() {

        // Reset the accumulated impulses for the limits
        mImpulseLowerLimit = 0.0f;
        mImpulseUpperLimit = 0.0f;

        // Wake up the two bodies of the joint
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    // Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi]
    protected float computeNormalizedAngle(float angle) {

        // Convert it into the range [-2*pi; 2*pi]
        angle = angle % Defaults.PI_TIMES_2;

        // Convert it into the range [-pi; pi]
        if (angle < -Defaults.PI) {
            return angle + Defaults.PI_TIMES_2;
        } else if (angle > Defaults.PI) {
            return angle - Defaults.PI_TIMES_2;
        } else {
            return angle;
        }
    }

    // Given an "inputAngle" in the range [-pi, pi], this method returns an
    // angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
    // two angle limits in arguments.
    protected float computeCorrespondingAngleNearLimits(float inputAngle, float lowerLimitAngle, float upperLimitAngle) {
        if (upperLimitAngle <= lowerLimitAngle) {
            return inputAngle;
        } else if (inputAngle > upperLimitAngle) {
            float diffToUpperLimit = Math.abs(computeNormalizedAngle(inputAngle - upperLimitAngle));
            float diffToLowerLimit = Math.abs(computeNormalizedAngle(inputAngle - lowerLimitAngle));
            return (diffToUpperLimit > diffToLowerLimit) ? (inputAngle - Defaults.PI_TIMES_2) : inputAngle;
        } else if (inputAngle < lowerLimitAngle) {
            float diffToUpperLimit = Math.abs(computeNormalizedAngle(upperLimitAngle - inputAngle));
            float diffToLowerLimit = Math.abs(computeNormalizedAngle(lowerLimitAngle - inputAngle));
            return (diffToUpperLimit > diffToLowerLimit) ? inputAngle : (inputAngle + Defaults.PI_TIMES_2);
        } else {
            return inputAngle;
        }
    }

    // Compute the current angle around the hinge axis
    protected float computeCurrentHingeAngle(Quaternion orientationBody1, Quaternion orientationBody2) {

        float hingeAngle;

        // Compute the current orientation difference between the two bodies
        Quaternion currentOrientationDiff = orientationBody2.operatorMultiply(orientationBody1.getInverse());
        currentOrientationDiff.normalize();

        // Compute the relative rotation considering the initial orientation difference
        Quaternion relativeRotation = currentOrientationDiff.operatorMultiply(mInitOrientationDifferenceInv);
        relativeRotation.normalize();

        // A quaternion q = [cos(theta/2); sin(theta/2) * rotAxis] where rotAxis is a unit
        // length vector. We can extract cos(theta/2) with q.w and we can extract |sin(theta/2)| with :
        // |sin(theta/2)| = q.getVectorV().length() since rotAxis is unit length. Note that any
        // rotation can be represented by a quaternion q and -q. Therefore, if the relative rotation
        // axis is not pointing in the same direction as the hinge axis, we use the rotation -q which
        // has the same |sin(theta/2)| value but the value cos(theta/2) is sign inverted. Some details
        // about this trick is explained in the source code of OpenTissue (http://www.opentissue.org).
        float cosHalfAngle = relativeRotation.w;
        float sinHalfAngleAbs = relativeRotation.getVectorV().length();

        // Compute the dot product of the relative rotation axis and the hinge axis
        float dotProduct = relativeRotation.getVectorV().dot(mA1);

        // If the relative rotation axis and the hinge axis are pointing the same direction
        if (dotProduct >= 0.0f) {
            hingeAngle = 2.0f * (float) Math.atan2(sinHalfAngleAbs, cosHalfAngle);
        } else {
            hingeAngle = 2.0f * (float) Math.atan2(sinHalfAngleAbs, -cosHalfAngle);
        }

        // Convert the angle from range [-2*pi; 2*pi] into the range [-pi; pi]
        hingeAngle = computeNormalizedAngle(hingeAngle);

        // Compute and return the corresponding angle near one the two limits
        return computeCorrespondingAngleNearLimits(hingeAngle, mLowerLimit, mUpperLimit);
    }

    // Constructor
    public HingeJoint(HingeJointInfo jointInfo) {
        super(jointInfo);

        mImpulseTranslation = new Vector3();
        mImpulseRotation = new Vector2();
        mImpulseLowerLimit = 0.0f;
        mImpulseUpperLimit = 0.0f;
        mImpulseMotor = 0.0f;
        mIsLimitEnabled = jointInfo.isLimitEnabled;
        mIsMotorEnabled = jointInfo.isMotorEnabled;
        mLowerLimit = jointInfo.minAngleLimit;
        mUpperLimit = jointInfo.maxAngleLimit;
        mIsLowerLimitViolated = false;
        mIsUpperLimitViolated = false;
        mMotorSpeed = jointInfo.motorSpeed;
        mMaxMotorTorque = jointInfo.maxMotorTorque;

        assert (mLowerLimit <= 0.0f && mLowerLimit >= -2.0f * Defaults.PI);
        assert (mUpperLimit >= 0.0f && mUpperLimit <= 2.0f * Defaults.PI);

        // Compute the local-space anchor point for each body
        Transform transform1 = mBody1.getTransform();
        Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = transform1.getInverse().operatorMultiply(jointInfo.anchorPointWorldSpace);
        mLocalAnchorPointBody2 = transform2.getInverse().operatorMultiply(jointInfo.anchorPointWorldSpace);

        // Compute the local-space hinge axis
        mHingeLocalAxisBody1 = transform1.getOrientation().getInverse().operatorMultiply(jointInfo.rotationAxisWorld);
        mHingeLocalAxisBody2 = transform2.getOrientation().getInverse().operatorMultiply(jointInfo.rotationAxisWorld);
        mHingeLocalAxisBody1.normalize();
        mHingeLocalAxisBody2.normalize();

        // Compute the inverse of the initial orientation difference between the two bodies
        mInitOrientationDifferenceInv = transform2.getOrientation().operatorMultiply(transform1.getOrientation().getInverse());
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
        mR1World = orientationBody1.operatorMultiply(mLocalAnchorPointBody1);
        mR2World = orientationBody2.operatorMultiply(mLocalAnchorPointBody2);

        // Compute the current angle around the hinge axis
        float hingeAngle = computeCurrentHingeAngle(orientationBody1, orientationBody2);

        // Check if the limit constraints are violated or not
        float lowerLimitError = hingeAngle - mLowerLimit;
        float upperLimitError = mUpperLimit - hingeAngle;
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

        // Compute vectors needed in the Jacobian
        mA1 = orientationBody1.operatorMultiply(mHingeLocalAxisBody1);
        Vector3 a2 = orientationBody2.operatorMultiply(mHingeLocalAxisBody2);
        mA1.normalize();
        a2.normalize();
        Vector3 b2 = a2.getOneUnitOrthogonalVector();
        Vector3 c2 = a2.cross(b2);
        mB2CrossA1 = b2.cross(mA1);
        mC2CrossA1 = c2.cross(mA1);

        // Compute the corresponding skew-symmetric matrices
        Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR1World);
        Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR2World);

        // Compute the inverse mass matrix K=JM^-1J^t for the 3 translation constraints (3x3 matrix)
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
            massMatrix.operatorAddEqual(
                    Matrix3x3.operatorMultiply(skewSymmetricMatrixU1, Matrix3x3.operatorMultiply(mI1, skewSymmetricMatrixU1.getTranspose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.operatorAddEqual(
                    Matrix3x3.operatorMultiply(skewSymmetricMatrixU2, Matrix3x3.operatorMultiply(mI2, skewSymmetricMatrixU2.getTranspose())));
        }
        mInverseMassMatrixTranslation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation = massMatrix.getInverse();
        }

        // Compute the bias "b" of the translation constraints
        mBTranslation.setToZero();
        float biasFactor = (BETA / constraintSolverData.timeStep);
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation = Vector3.operatorMultiply(
                    biasFactor, Vector3.operatorSubtract(Vector3.operatorSubtract(Vector3.operatorAdd(x2, mR2World), x1), mR1World));
        }

        // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
        Vector3 I1B2CrossA1 = new Vector3();
        Vector3 I1C2CrossA1 = new Vector3();
        Vector3 I2B2CrossA1 = new Vector3();
        Vector3 I2C2CrossA1 = new Vector3();
        if (mBody1.isMotionEnabled()) {
            I1B2CrossA1 = Matrix3x3.operatorMultiply(mI1, mB2CrossA1);
            I1C2CrossA1 = Matrix3x3.operatorMultiply(mI1, mC2CrossA1);
        }
        if (mBody2.isMotionEnabled()) {
            I2B2CrossA1 = Matrix3x3.operatorMultiply(mI2, mB2CrossA1);
            I2C2CrossA1 = Matrix3x3.operatorMultiply(mI2, mC2CrossA1);
        }
        float el11 = mB2CrossA1.dot(I1B2CrossA1) + mB2CrossA1.dot(I2B2CrossA1);
        float el12 = mB2CrossA1.dot(I1C2CrossA1) + mB2CrossA1.dot(I2C2CrossA1);
        float el21 = mC2CrossA1.dot(I1B2CrossA1) + mC2CrossA1.dot(I2B2CrossA1);
        float el22 = mC2CrossA1.dot(I1C2CrossA1) + mC2CrossA1.dot(I2C2CrossA1);
        Matrix2x2 matrixKRotation = new Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixRotation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation = matrixKRotation.getInverse();
        }

        // Compute the bias "b" of the rotation constraints
        mBRotation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBRotation = Vector2.operatorMultiply(biasFactor, new Vector2(mA1.dot(b2), mA1.dot(c2)));
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

        // If the motor or limits are enabled
        if (mIsMotorEnabled || (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated))) {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits and motor (1x1 matrix)
            mInverseMassMatrixLimitMotor = 0.0f;
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.operatorMultiply(mI1, mA1));
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.operatorMultiply(mI2, mA1));
            }
            mInverseMassMatrixLimitMotor = (mInverseMassMatrixLimitMotor > 0.0f) ? 1.0f / mInverseMassMatrixLimitMotor : 0.0f;

            if (mIsLimitEnabled) {

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

        // Get the inverse mass and inverse inertia tensors of the bodies
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();

        // Compute the impulse P=J^T * lambda for the 2 rotation constraints
        Vector3 rotationImpulse = Vector3.operatorSubtract(
                Vector3.operatorMultiply(Vector3.operatorNegative(mB2CrossA1), mImpulseRotation.x),
                Vector3.operatorMultiply(mC2CrossA1, mImpulseRotation.y));

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
        Vector3 limitsImpulse = Vector3.operatorMultiply(mImpulseUpperLimit - mImpulseLowerLimit, mA1);

        // Compute the impulse P=J^T * lambda for the motor constraint
        Vector3 motorImpulse = Vector3.operatorMultiply(-mImpulseMotor, mA1);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 translation constraints
            Vector3 linearImpulseBody1 = Vector3.operatorNegative(mImpulseTranslation);
            Vector3 angularImpulseBody1 = mImpulseTranslation.cross(mR1World);

            // Compute the impulse P=J^T * lambda for the 2 rotation constraints
            angularImpulseBody1.operatorAddEqual(rotationImpulse);

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            angularImpulseBody1.operatorAddEqual(limitsImpulse);

            // Compute the impulse P=J^T * lambda for the motor constraint
            angularImpulseBody1.operatorAddEqual(motorImpulse);

            // Apply the impulse to the body
            v1.operatorAddEqual(Vector3.operatorMultiply(inverseMassBody1, linearImpulseBody1));
            w1.operatorAddEqual(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 translation constraints
            Vector3 linearImpulseBody2 = mImpulseTranslation;
            Vector3 angularImpulseBody2 = Vector3.operatorNegative(mImpulseTranslation.cross(mR2World));

            // Compute the impulse P=J^T * lambda for the 2 rotation constraints
            angularImpulseBody2.operatorAddEqual(Vector3.operatorNegative(rotationImpulse));

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            angularImpulseBody2.operatorAddEqual(Vector3.operatorNegative(limitsImpulse));

            // Compute the impulse P=J^T * lambda for the motor constraint
            angularImpulseBody2.operatorAddEqual(Vector3.operatorNegative(motorImpulse));

            // Apply the impulse to the body
            v2.operatorAddEqual(Vector3.operatorMultiply(inverseMassBody2, linearImpulseBody2));
            w2.operatorAddEqual(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
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

        // Get the inverse mass and inverse inertia tensors of the bodies
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();

        /**
         * --------------- Translation Constraints ---------------
         */
        // Compute J*v
        Vector3 JvTranslation = Vector3.operatorSubtract(
                Vector3.operatorSubtract(Vector3.operatorAdd(v2, w2.cross(mR2World)), v1), w1.cross(mR1World));

        // Compute the Lagrange multiplier lambda
        Vector3 deltaLambdaTranslation = Matrix3x3.operatorMultiply(
                mInverseMassMatrixTranslation, Vector3.operatorSubtract(Vector3.operatorNegative(JvTranslation), mBTranslation));
        mImpulseTranslation.operatorAddEqual(deltaLambdaTranslation);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda
            Vector3 linearImpulseBody1 = Vector3.operatorNegative(deltaLambdaTranslation);
            Vector3 angularImpulseBody1 = deltaLambdaTranslation.cross(mR1World);

            // Apply the impulse to the body
            v1.operatorAddEqual(Vector3.operatorMultiply(inverseMassBody1, linearImpulseBody1));
            w1.operatorAddEqual(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda
            Vector3 linearImpulseBody2 = deltaLambdaTranslation;
            Vector3 angularImpulseBody2 = Vector3.operatorNegative(deltaLambdaTranslation.cross(mR2World));

            // Apply the impulse to the body
            v2.operatorAddEqual(Vector3.operatorMultiply(inverseMassBody2, linearImpulseBody2));
            w2.operatorAddEqual(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
        }

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute J*v for the 2 rotation constraints
        Vector2 JvRotation = new Vector2(-mB2CrossA1.dot(w1) + mB2CrossA1.dot(w2), -mC2CrossA1.dot(w1) + mC2CrossA1.dot(w2));

        // Compute the Lagrange multiplier lambda for the 2 rotation constraints
        Vector2 deltaLambdaRotation = Matrix2x2.operatorMultiply(
                mInverseMassMatrixRotation, Vector2.operatorSubtract(Vector2.operatorNegative(JvRotation), mBRotation));
        mImpulseRotation.operatorAddEqual(deltaLambdaRotation);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 rotation constraints
            Vector3 angularImpulseBody1 = Vector3.operatorSubtract(
                    Vector3.operatorMultiply(Vector3.operatorNegative(mB2CrossA1), deltaLambdaRotation.x),
                    Vector3.operatorMultiply(mC2CrossA1, deltaLambdaRotation.y));

            // Apply the impulse to the body
            w1.operatorAddEqual(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 rotation constraints
            Vector3 angularImpulseBody2 = Vector3.operatorAdd(
                    Vector3.operatorMultiply(mB2CrossA1, deltaLambdaRotation.x),
                    Vector3.operatorMultiply(mC2CrossA1, deltaLambdaRotation.y));

            // Apply the impulse to the body
            w2.operatorAddEqual(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
        }

        /**
         * --------------- Limits Constraints ---------------
         */
        if (mIsLimitEnabled) {

            // If the lower limit is violated
            if (mIsLowerLimitViolated) {

                // Compute J*v for the lower limit constraint
                float JvLowerLimit = Vector3.operatorSubtract(w2, w1).dot(mA1);

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                float deltaLambdaLower = mInverseMassMatrixLimitMotor * (-JvLowerLimit - mBLowerLimit);
                float lambdaTemp = mImpulseLowerLimit;
                mImpulseLowerLimit = (float) Math.max(mImpulseLowerLimit + deltaLambdaLower, 0.0f);
                deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    Vector3 angularImpulseBody1 = Vector3.operatorMultiply(-deltaLambdaLower, mA1);

                    // Apply the impulse to the body
                    w1.operatorAddEqual(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    Vector3 angularImpulseBody2 = Vector3.operatorMultiply(deltaLambdaLower, mA1);

                    // Apply the impulse to the body
                    w2.operatorAddEqual(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
                }
            }

            // If the upper limit is violated
            if (mIsUpperLimitViolated) {

                // Compute J*v for the upper limit constraint
                float JvUpperLimit = -Vector3.operatorSubtract(w2, w1).dot(mA1);

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                float deltaLambdaUpper = mInverseMassMatrixLimitMotor * (-JvUpperLimit - mBUpperLimit);
                float lambdaTemp = mImpulseUpperLimit;
                mImpulseUpperLimit = (float) Math.max(mImpulseUpperLimit + deltaLambdaUpper, 0.0f);
                deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    Vector3 angularImpulseBody1 = Vector3.operatorMultiply(deltaLambdaUpper, mA1);

                    // Apply the impulse to the body
                    w1.operatorAddEqual(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    Vector3 angularImpulseBody2 = Vector3.operatorMultiply(-deltaLambdaUpper, mA1);

                    // Apply the impulse to the body
                    w2.operatorAddEqual(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
                }
            }
        }

        /**
         * --------------- Motor ---------------
         */
        // If the motor is enabled
        if (mIsMotorEnabled) {

            // Compute J*v for the motor
            float JvMotor = mA1.dot(Vector3.operatorSubtract(w1, w2));

            // Compute the Lagrange multiplier lambda for the motor
            float maxMotorImpulse = mMaxMotorTorque * constraintSolverData.timeStep;
            float deltaLambdaMotor = mInverseMassMatrixLimitMotor * (-JvMotor - mMotorSpeed);
            float lambdaTemp = mImpulseMotor;
            mImpulseMotor = Mathematics.Clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mImpulseMotor - lambdaTemp;

            if (mBody1.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                Vector3 angularImpulseBody1 = Vector3.operatorMultiply(-deltaLambdaMotor, mA1);

                // Apply the impulse to the body
                w1.operatorAddEqual(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
            }
            if (mBody2.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                Vector3 angularImpulseBody2 = Vector3.operatorMultiply(deltaLambdaMotor, mA1);

                // Apply the impulse to the body
                w2.operatorAddEqual(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
            }
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
        mR1World = q1.operatorMultiply(mLocalAnchorPointBody1);
        mR2World = q2.operatorMultiply(mLocalAnchorPointBody2);

        // Compute the current angle around the hinge axis
        float hingeAngle = computeCurrentHingeAngle(q1, q2);

        // Check if the limit constraints are violated or not
        float lowerLimitError = hingeAngle - mLowerLimit;
        float upperLimitError = mUpperLimit - hingeAngle;
        mIsLowerLimitViolated = lowerLimitError <= 0.0f;
        mIsUpperLimitViolated = upperLimitError <= 0.0f;

        // Compute vectors needed in the Jacobian
        mA1 = q1.operatorMultiply(mHingeLocalAxisBody1);
        Vector3 a2 = q2.operatorMultiply(mHingeLocalAxisBody2);
        mA1.normalize();
        a2.normalize();
        Vector3 b2 = a2.getOneUnitOrthogonalVector();
        Vector3 c2 = a2.cross(b2);
        mB2CrossA1 = b2.cross(mA1);
        mC2CrossA1 = c2.cross(mA1);

        // Compute the corresponding skew-symmetric matrices
        Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR1World);
        Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR2World);

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
            massMatrix.operatorAddEqual(
                    Matrix3x3.operatorMultiply(skewSymmetricMatrixU1, Matrix3x3.operatorMultiply(mI1, skewSymmetricMatrixU1.getTranspose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.operatorAddEqual(
                    Matrix3x3.operatorMultiply(skewSymmetricMatrixU2, Matrix3x3.operatorMultiply(mI2, skewSymmetricMatrixU2.getTranspose())));
        }
        mInverseMassMatrixTranslation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation = massMatrix.getInverse();
        }

        // Compute position error for the 3 translation constraints
        Vector3 errorTranslation = Vector3.operatorSubtract(
                Vector3.operatorSubtract(Vector3.operatorAdd(x2, mR2World), x1), mR1World);

        // Compute the Lagrange multiplier lambda
        Vector3 lambdaTranslation = Matrix3x3.operatorMultiply(
                mInverseMassMatrixTranslation, (Vector3.operatorNegative(errorTranslation)));

        // Apply the impulse to the bodies of the joint
        if (mBody1.isMotionEnabled()) {

            // Compute the impulse
            Vector3 linearImpulseBody1 = Vector3.operatorNegative(lambdaTranslation);
            Vector3 angularImpulseBody1 = lambdaTranslation.cross(mR1World);

            // Compute the pseudo velocity
            Vector3 v1 = Vector3.operatorMultiply(inverseMassBody1, linearImpulseBody1);
            Vector3 w1 = Matrix3x3.operatorMultiply(mI1, angularImpulseBody1);

            // Update the body position/orientation
            x1.operatorAddEqual(v1);
            q1.operatorAddEqual(new Quaternion(0.0f, w1).operatorMultiply(q1).operatorMultiply(0.5f));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse
            Vector3 linearImpulseBody2 = lambdaTranslation;
            Vector3 angularImpulseBody2 = Vector3.operatorNegative(lambdaTranslation.cross(mR2World));

            // Compute the pseudo velocity
            Vector3 v2 = Vector3.operatorMultiply(inverseMassBody2, linearImpulseBody2);
            Vector3 w2 = Matrix3x3.operatorMultiply(mI2, angularImpulseBody2);

            // Update the body position/orientation
            x2.operatorAddEqual(v2);
            q2.operatorAddEqual(new Quaternion(0.0f, w2).operatorMultiply(q2).operatorMultiply(0.5f));
            q2.normalize();
        }

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
        Vector3 I1B2CrossA1 = new Vector3();
        Vector3 I1C2CrossA1 = new Vector3();
        Vector3 I2B2CrossA1 = new Vector3();
        Vector3 I2C2CrossA1 = new Vector3();
        if (mBody1.isMotionEnabled()) {
            I1B2CrossA1 = Matrix3x3.operatorMultiply(mI1, mB2CrossA1);
            I1C2CrossA1 = Matrix3x3.operatorMultiply(mI1, mC2CrossA1);
        }
        if (mBody2.isMotionEnabled()) {
            I2B2CrossA1 = Matrix3x3.operatorMultiply(mI2, mB2CrossA1);
            I2C2CrossA1 = Matrix3x3.operatorMultiply(mI2, mC2CrossA1);
        }
        float el11 = mB2CrossA1.dot(I1B2CrossA1) + mB2CrossA1.dot(I2B2CrossA1);
        float el12 = mB2CrossA1.dot(I1C2CrossA1) + mB2CrossA1.dot(I2C2CrossA1);
        float el21 = mC2CrossA1.dot(I1B2CrossA1) + mC2CrossA1.dot(I2B2CrossA1);
        float el22 = mC2CrossA1.dot(I1C2CrossA1) + mC2CrossA1.dot(I2C2CrossA1);
        Matrix2x2 matrixKRotation = new Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixRotation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation = matrixKRotation.getInverse();
        }

        // Compute the position error for the 3 rotation constraints
        Vector2 errorRotation = new Vector2(mA1.dot(b2), mA1.dot(c2));

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        Vector2 lambdaRotation = Matrix2x2.operatorMultiply(
                mInverseMassMatrixRotation, Vector2.operatorNegative(errorRotation));

        // Apply the impulse to the bodies of the joint
        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody1 = Vector3.operatorSubtract(
                    Vector3.operatorMultiply(Vector3.operatorNegative(mB2CrossA1), lambdaRotation.x),
                    Vector3.operatorMultiply(mC2CrossA1, lambdaRotation.y));

            // Compute the pseudo velocity
            Vector3 w1 = Matrix3x3.operatorMultiply(mI1, angularImpulseBody1);

            // Update the body position/orientation
            q1.operatorAddEqual(new Quaternion(0.0f, w1).operatorMultiply(q1).operatorMultiply(0.5f));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse
            Vector3 angularImpulseBody2 = Vector3.operatorAdd(
                    Vector3.operatorMultiply(mB2CrossA1, lambdaRotation.x),
                    Vector3.operatorMultiply(mC2CrossA1, lambdaRotation.y));

            // Compute the pseudo velocity
            Vector3 w2 = Matrix3x3.operatorMultiply(mI2, angularImpulseBody2);

            // Update the body position/orientation
            q2.operatorAddEqual(new Quaternion(0.0f, w2).operatorMultiply(q2).operatorMultiply(0.5f));
            q2.normalize();
        }

        /**
         * --------------- Limits Constraints ---------------
         */
        if (mIsLimitEnabled) {

            if (mIsLowerLimitViolated || mIsUpperLimitViolated) {

                // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
                mInverseMassMatrixLimitMotor = 0.0f;
                if (mBody1.isMotionEnabled()) {
                    mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.operatorMultiply(mI1, mA1));
                }
                if (mBody2.isMotionEnabled()) {
                    mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.operatorMultiply(mI2, mA1));
                }
                mInverseMassMatrixLimitMotor = (mInverseMassMatrixLimitMotor > 0.0f) ? 1.0f / mInverseMassMatrixLimitMotor : 0.0f;
            }

            // If the lower limit is violated
            if (mIsLowerLimitViolated) {

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                float lambdaLowerLimit = mInverseMassMatrixLimitMotor * (-lowerLimitError);

                // Apply the impulse to the bodies of the joint
                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda
                    Vector3 angularImpulseBody1 = Vector3.operatorMultiply(-lambdaLowerLimit, mA1);

                    // Compute the pseudo velocity
                    Vector3 w1 = Matrix3x3.operatorMultiply(mI1, angularImpulseBody1);

                    // Update the body position/orientation
                    q1.operatorAddEqual(new Quaternion(0.0f, w1).operatorMultiply(q1).operatorMultiply(0.5f));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda
                    Vector3 angularImpulseBody2 = Vector3.operatorMultiply(lambdaLowerLimit, mA1);

                    // Compute the pseudo velocity
                    Vector3 w2 = Matrix3x3.operatorMultiply(mI2, angularImpulseBody2);

                    // Update the body position/orientation
                    q2.operatorAddEqual(new Quaternion(0.0f, w2).operatorMultiply(q2).operatorMultiply(0.5f));
                    q2.normalize();
                }
            }

            // If the upper limit is violated
            if (mIsUpperLimitViolated) {

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                float lambdaUpperLimit = mInverseMassMatrixLimitMotor * (-upperLimitError);

                // Apply the impulse to the bodies of the joint
                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda
                    Vector3 angularImpulseBody1 = Vector3.operatorMultiply(lambdaUpperLimit, mA1);

                    // Compute the pseudo velocity
                    Vector3 w1 = Matrix3x3.operatorMultiply(mI1, angularImpulseBody1);

                    // Update the body position/orientation
                    q1.operatorAddEqual(new Quaternion(0.0f, w1).operatorMultiply(q1).operatorMultiply(0.5f));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda
                    Vector3 angularImpulseBody2 = Vector3.operatorMultiply(-lambdaUpperLimit, mA1);

                    // Compute the pseudo velocity
                    Vector3 w2 = Matrix3x3.operatorMultiply(mI2, angularImpulseBody2);

                    // Update the body position/orientation
                    q2.operatorAddEqual(new Quaternion(0.0f, w2).operatorMultiply(q2).operatorMultiply(0.5f));
                    q2.normalize();
                }
            }
        }
    }

    // Return true if the limits or the joint are enabled
    public boolean isLimitEnabled() {
        return mIsLimitEnabled;
    }

    // Return true if the motor of the joint is enabled
    public boolean isMotorEnabled() {
        return mIsMotorEnabled;
    }

    // Return the minimum angle limit
    public float getMinAngleLimit() {
        return mLowerLimit;
    }

    // Return the maximum angle limit
    public float getMaxAngleLimit() {
        return mUpperLimit;
    }

    // Return the motor speed
    public float getMotorSpeed() {
        return mMotorSpeed;
    }

    // Return the maximum motor torque
    public float getMaxMotorTorque() {
        return mMaxMotorTorque;
    }

    // Return the intensity of the current torque applied for the joint motor
    public float getMotorTorque(float timeStep) {
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

    // Set the minimum angle limit
    public void setMinAngleLimit(float lowerLimit) {

        assert (mLowerLimit <= 0.0f && mLowerLimit >= -2.0 * Defaults.PI);

        if (lowerLimit != mLowerLimit) {

            mLowerLimit = lowerLimit;

            // Reset the limits
            resetLimits();
        }
    }

    // Set the maximum angle limit
    public void setMaxAngleLimit(float upperLimit) {

        assert (upperLimit >= 0.0f && upperLimit <= 2.0f * Defaults.PI);

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

    // Set the maximum motor torque
    public void setMaxMotorTorque(float maxMotorTorque) {

        if (maxMotorTorque != mMaxMotorTorque) {

            assert (mMaxMotorTorque >= 0.0f);
            mMaxMotorTorque = maxMotorTorque;

            // Wake up the two bodies of the joint
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

}
