package net.smert.jreactphysics3d.constraint;

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
 * This class represents a slider joint. This joint has a one degree of freedom. It only allows relative translation of
 * the bodies along a single direction and no rotation.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class SliderJoint extends Joint {

    // Beta value for the position correction bias factor
    private static final float BETA = 0.2f;

    // Anchor point of body 1 (in local-space coordinates of body 1)
    private Vector3 mLocalAnchorPointBody1;

    // Anchor point of body 2 (in local-space coordinates of body 2)
    private Vector3 mLocalAnchorPointBody2;

    // Slider axis (in local-space coordinates of body 1)
    private Vector3 mSliderAxisBody1;

    // Inertia tensor of body 1 (in world-space coordinates)
    private Matrix3x3 mI1;

    // Inertia tensor of body 2 (in world-space coordinates)
    private Matrix3x3 mI2;

    // Inverse of the initial orientation difference between the two bodies
    private Quaternion mInitOrientationDifferenceInv;

    // First vector orthogonal to the slider axis local-space of body 1
    private Vector3 mN1;

    // Second vector orthogonal to the slider axis and mN1 in local-space of body 1
    private Vector3 mN2;

    // Vector r1 in world-space coordinates
    private Vector3 mR1;

    // Vector r2 in world-space coordinates
    private Vector3 mR2;

    // Cross product of r2 and n1
    private Vector3 mR2CrossN1;

    // Cross product of r2 and n2
    private Vector3 mR2CrossN2;

    // Cross product of r2 and the slider axis
    private Vector3 mR2CrossSliderAxis;

    // Cross product of vector (r1 + u) and n1
    private Vector3 mR1PlusUCrossN1;

    // Cross product of vector (r1 + u) and n2
    private Vector3 mR1PlusUCrossN2;

    // Cross product of vector (r1 + u) and the slider axis
    private Vector3 mR1PlusUCrossSliderAxis;

    // Bias of the 2 translation constraints
    private Vector2 mBTranslation;

    // Bias of the 3 rotation constraints
    private Vector3 mBRotation;

    // Bias of the lower limit constraint
    private float mBLowerLimit;

    // Bias of the upper limit constraint
    private float mBUpperLimit;

    // Inverse of mass matrix K=JM^-1J^t for the translation constraint (2x2 matrix)
    private Matrix2x2 mInverseMassMatrixTranslationConstraint;

    // Inverse of mass matrix K=JM^-1J^t for the rotation constraint (3x3 matrix)
    private Matrix3x3 mInverseMassMatrixRotationConstraint;

    // Inverse of mass matrix K=JM^-1J^t for the upper and lower limit constraints (1x1 matrix)
    private float mInverseMassMatrixLimit;

    // Inverse of mass matrix K=JM^-1J^t for the motor
    private float mInverseMassMatrixMotor;

    // Accumulated impulse for the 2 translation constraints
    private Vector2 mImpulseTranslation;

    // Accumulated impulse for the 3 rotation constraints
    private Vector3 mImpulseRotation;

    // Accumulated impulse for the lower limit constraint
    private float mImpulseLowerLimit;

    // Accumulated impulse for the upper limit constraint
    private float mImpulseUpperLimit;

    // Accumulated impulse for the motor
    private float mImpulseMotor;

    // True if the slider limits are enabled
    private boolean mIsLimitEnabled;

    // True if the motor of the joint in enabled
    private boolean mIsMotorEnabled;

    // Slider axis in world-space coordinates
    private Vector3 mSliderAxisWorld;

    // Lower limit (minimum translation distance)
    private float mLowerLimit;

    // Upper limit (maximum translation distance)
    private float mUpperLimit;

    // True if the lower limit is violated
    private boolean mIsLowerLimitViolated;

    // True if the upper limit is violated
    private boolean mIsUpperLimitViolated;

    // Motor speed
    private float mMotorSpeed;

    // Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
    private float mMaxMotorForce;

    // Reset the limits
    protected void resetLimits() {

        // Reset the accumulated impulses for the limits
        mImpulseLowerLimit = 0.0f;
        mImpulseUpperLimit = 0.0f;

        // Wake up the two bodies of the joint
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    // Constructor
    public SliderJoint(SliderJointInfo jointInfo) {
        super(jointInfo);

        mImpulseTranslation = new Vector2();
        mImpulseRotation = new Vector3();
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
        mLocalAnchorPointBody1 = transform1.getInverse().operatorMultiply(jointInfo.anchorPointWorldSpace);
        mLocalAnchorPointBody2 = transform2.getInverse().operatorMultiply(jointInfo.anchorPointWorldSpace);

        // Compute the inverse of the initial orientation difference between the two bodies
        mInitOrientationDifferenceInv = new Quaternion(transform2.getOrientation()).multiply(new Quaternion(transform1.getOrientation()).inverse());
        mInitOrientationDifferenceInv.normalize();
        mInitOrientationDifferenceInv.inverse();

        // Compute the slider axis in local-space of body 1
        new Quaternion(mBody1.getTransform().getOrientation()).inverse().multiply(jointInfo.sliderAxisWorldSpace, mSliderAxisBody1);
        mSliderAxisBody1.normalize();
    }

    // Initialize before solving the constraint
    @Override
    public void initBeforeSolve(ConstraintSolverData constraintSolverData) {

        // Initialize the bodies index in the veloc ity array
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

        // Vector from body center to the anchor point
        orientationBody1.multiply(mLocalAnchorPointBody1, mR1);
        orientationBody2.multiply(mLocalAnchorPointBody2, mR2);

        // Compute the vector u (difference between anchor points)
        Vector3 u = new Vector3(new Vector3(new Vector3(x2).add(mR2)).subtract(x1)).subtract(mR1);

        // Compute the two orthogonal vectors to the slider axis in world-space
        orientationBody1.multiply(mSliderAxisBody1, mSliderAxisWorld);
        mSliderAxisWorld.normalize();
        mN1 = new Vector3(mSliderAxisWorld).setUnitOrthogonal();
        mN2 = new Vector3(mSliderAxisWorld).cross(mN1);

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
        mR2CrossN1 = new Vector3(mR2).cross(mN1);
        mR2CrossN2 = new Vector3(mR2).cross(mN2);
        mR2CrossSliderAxis = new Vector3(mR2).cross(mSliderAxisWorld);
        Vector3 r1PlusU = new Vector3(mR1).add(u);
        mR1PlusUCrossN1 = new Vector3(r1PlusU).cross(mN1);
        mR1PlusUCrossN2 = new Vector3(r1PlusU).cross(mN2);
        mR1PlusUCrossSliderAxis = new Vector3(r1PlusU).cross(mSliderAxisWorld);

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
        // constraints (2x2 matrix)
        float sumInverseMass = 0.0f;
        Vector3 I1R1PlusUCrossN1 = new Vector3();
        Vector3 I1R1PlusUCrossN2 = new Vector3();
        Vector3 I2R2CrossN1 = new Vector3();
        Vector3 I2R2CrossN2 = new Vector3();
        if (mBody1.isMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1 = Matrix3x3.operatorMultiply(mI1, mR1PlusUCrossN1);
            I1R1PlusUCrossN2 = Matrix3x3.operatorMultiply(mI1, mR1PlusUCrossN2);
        }
        if (mBody2.isMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1 = Matrix3x3.operatorMultiply(mI2, mR2CrossN1);
            I2R2CrossN2 = Matrix3x3.operatorMultiply(mI2, mR2CrossN2);
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
        mBTranslation.zero();
        float biasFactor = (BETA / constraintSolverData.timeStep);
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation.setX(u.dot(mN1));
            mBTranslation.setY(u.dot(mN2));
            mBTranslation.multiply(biasFactor);
        }

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mInverseMassMatrixRotationConstraint.setToZero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.operatorAddEqual(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.operatorAddEqual(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
        }

        // Compute the bias "b" of the rotation constraint
        mBRotation.zero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            Quaternion currentOrientationDifference = new Quaternion(orientationBody2).multiply(new Quaternion(orientationBody1).inverse());
            currentOrientationDifference.normalize();
            Quaternion qError = new Quaternion(currentOrientationDifference).multiply(mInitOrientationDifferenceInv);
            Vector3 qErrorV = new Vector3();
            qError.getVectorV(qErrorV);
            mBRotation = qErrorV.multiply(biasFactor * 2.0f);
        }

        // If the limits are enabled
        if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated)) {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            mInverseMassMatrixLimit = 0.0f;
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixLimit += mBody1.getMassInverse()
                        + mR1PlusUCrossSliderAxis.dot(Matrix3x3.operatorMultiply(mI1, mR1PlusUCrossSliderAxis));
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixLimit += mBody2.getMassInverse()
                        + mR2CrossSliderAxis.dot(Matrix3x3.operatorMultiply(mI2, mR2CrossSliderAxis));
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
            mImpulseTranslation.zero();
            mImpulseRotation.zero();
            mImpulseLowerLimit = 0.0f;
            mImpulseUpperLimit = 0.0f;
            mImpulseMotor = 0.0f;
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

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
        float impulseLimits = mImpulseUpperLimit - mImpulseLowerLimit;
        Vector3 linearImpulseLimits = new Vector3(mSliderAxisWorld).multiply(impulseLimits);

        // Compute the impulse P=J^T * lambda for the motor constraint
        Vector3 impulseMotor = new Vector3(mSliderAxisWorld).multiply(mImpulseMotor);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody1 = new Vector3(
                    new Vector3(new Vector3(mN1).invert()).multiply(mImpulseTranslation.getX())).subtract(
                            new Vector3(mN2).multiply(mImpulseTranslation.getY()));
            Vector3 angularImpulseBody1 = new Vector3(
                    new Vector3(new Vector3(mR1PlusUCrossN1).invert()).multiply(mImpulseTranslation.getX())).subtract(
                            new Vector3(mR1PlusUCrossN2).multiply(mImpulseTranslation.getY()));

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            angularImpulseBody1.add(new Vector3(mImpulseRotation).invert());

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            linearImpulseBody1.add(linearImpulseLimits);
            angularImpulseBody1.add(new Vector3(mR1PlusUCrossSliderAxis).multiply(impulseLimits));

            // Compute the impulse P=J^T * lambda for the motor constraint
            linearImpulseBody1.add(impulseMotor);

            // Apply the impulse to the body
            v1.add(new Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            w1.add(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody2 = new Vector3(
                    new Vector3(new Vector3(mN1).invert()).multiply(mImpulseTranslation.getX())).add(
                            new Vector3(mN2).multiply(mImpulseTranslation.getY()));
            Vector3 angularImpulseBody2 = new Vector3(
                    new Vector3(new Vector3(mR2CrossN1).invert()).multiply(mImpulseTranslation.getX())).add(
                            new Vector3(mR2CrossN2).multiply(mImpulseTranslation.getY()));

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            angularImpulseBody2.add(mImpulseRotation);

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            linearImpulseBody2.add(new Vector3(linearImpulseLimits).invert());
            angularImpulseBody2.add(new Vector3(mR2CrossSliderAxis).multiply(-impulseLimits));

            // Compute the impulse P=J^T * lambda for the motor constraint
            linearImpulseBody2.add(new Vector3(impulseMotor).invert());

            // Apply the impulse to the body
            v2.add(new Vector3(linearImpulseBody2).multiply(inverseMassBody2));
            w2.add(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
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
        // Compute J*v for the 2 translation constraints
        float el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) + mN1.dot(v2) + w2.dot(mR2CrossN1);
        float el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) + mN2.dot(v2) + w2.dot(mR2CrossN2);
        Vector2 JvTranslation = new Vector2(el1, el2);

        // Compute the Lagrange multiplier lambda for the 2 translation constraints
        Vector2 deltaLambda = Matrix2x2.operatorMultiply(
                mInverseMassMatrixTranslationConstraint, new Vector2(JvTranslation).invert().subtract(mBTranslation));
        mImpulseTranslation.add(deltaLambda);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody1 = new Vector3(
                    new Vector3(new Vector3(mN1).invert()).multiply(deltaLambda.getX())).subtract(
                            new Vector3(mN2).multiply(deltaLambda.getY()));
            Vector3 angularImpulseBody1 = new Vector3(
                    new Vector3(new Vector3(mR1PlusUCrossN1).invert()).multiply(deltaLambda.getX())).subtract(
                            new Vector3(mR1PlusUCrossN2).multiply(deltaLambda.getY()));

            // Apply the impulse to the body
            v1.add(new Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            w1.add(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody2 = new Vector3(
                    new Vector3(new Vector3(mN1).invert()).multiply(deltaLambda.getX())).add(
                            new Vector3(mN2).multiply(deltaLambda.getY()));
            Vector3 angularImpulseBody2 = new Vector3(
                    new Vector3(new Vector3(mR2CrossN1).invert()).multiply(deltaLambda.getX())).add(
                            new Vector3(mR2CrossN2).multiply(deltaLambda.getY()));

            // Apply the impulse to the body
            v2.add(new Vector3(linearImpulseBody2).multiply(inverseMassBody2));
            w2.add(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
        }

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute J*v for the 3 rotation constraints
        Vector3 JvRotation = new Vector3(w2).subtract(w1);

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        Vector3 deltaLambda2 = Matrix3x3.operatorMultiply(
                mInverseMassMatrixRotationConstraint, new Vector3(new Vector3(JvRotation).invert()).subtract(mBRotation));
        mImpulseRotation.add(deltaLambda2);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody1 = new Vector3(deltaLambda2).invert();

            // Apply the impulse to the body
            w1.add(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody2 = deltaLambda2;

            // Apply the impulse to the body
            w2.add(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
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
                    Vector3 linearImpulseBody1 = new Vector3(mSliderAxisWorld).multiply(-deltaLambdaLower);
                    Vector3 angularImpulseBody1 = new Vector3(mR1PlusUCrossSliderAxis).multiply(-deltaLambdaLower);

                    // Apply the impulse to the body
                    v1.add(new Vector3(linearImpulseBody1).multiply(inverseMassBody1));
                    w1.add(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    Vector3 linearImpulseBody2 = new Vector3(mSliderAxisWorld).multiply(deltaLambdaLower);
                    Vector3 angularImpulseBody2 = new Vector3(mR2CrossSliderAxis).multiply(deltaLambdaLower);

                    // Apply the impulse to the body
                    v2.add(new Vector3(linearImpulseBody2).multiply(inverseMassBody2));
                    w2.add(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
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
                    Vector3 linearImpulseBody1 = new Vector3(mSliderAxisWorld).multiply(deltaLambdaUpper);
                    Vector3 angularImpulseBody1 = new Vector3(mR1PlusUCrossSliderAxis).multiply(deltaLambdaUpper);

                    // Apply the impulse to the body
                    v1.add(new Vector3(linearImpulseBody1).multiply(inverseMassBody1));
                    w1.add(Matrix3x3.operatorMultiply(mI1, angularImpulseBody1));
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    Vector3 linearImpulseBody2 = new Vector3(mSliderAxisWorld).multiply(-deltaLambdaUpper);
                    Vector3 angularImpulseBody2 = new Vector3(mR2CrossSliderAxis).multiply(-deltaLambdaUpper);

                    // Apply the impulse to the body
                    v2.add(new Vector3(linearImpulseBody2).multiply(inverseMassBody2));
                    w2.add(Matrix3x3.operatorMultiply(mI2, angularImpulseBody2));
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
            mImpulseMotor = Mathematics.Clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mImpulseMotor - lambdaTemp;

            if (mBody1.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                Vector3 linearImpulseBody1 = new Vector3(mSliderAxisWorld).multiply(deltaLambdaMotor);

                // Apply the impulse to the body
                v1.add(new Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            }
            if (mBody2.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                Vector3 linearImpulseBody2 = new Vector3(mSliderAxisWorld).multiply(-deltaLambdaMotor);

                // Apply the impulse to the body
                v2.add(new Vector3(linearImpulseBody2).multiply(inverseMassBody2));
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

        // Recompute the inertia tensor of bodies
        mI1 = mBody1.getInertiaTensorInverseWorld();
        mI2 = mBody2.getInertiaTensorInverseWorld();

        // Vector from body center to the anchor point
        q1.multiply(mLocalAnchorPointBody1, mR1);
        q2.multiply(mLocalAnchorPointBody2, mR2);

        // Compute the vector u (difference between anchor points)
        Vector3 u = new Vector3(new Vector3(new Vector3(x2).add(mR2)).subtract(x1)).subtract(mR1);

        // Compute the two orthogonal vectors to the slider axis in world-space
        q1.multiply(mSliderAxisBody1, mSliderAxisWorld);
        mSliderAxisWorld.normalize();
        mN1 = new Vector3(mSliderAxisWorld).setUnitOrthogonal();
        mN2 = new Vector3(mSliderAxisWorld).cross(mN1);

        // Check if the limit constraints are violated or not
        float uDotSliderAxis = u.dot(mSliderAxisWorld);
        float lowerLimitError = uDotSliderAxis - mLowerLimit;
        float upperLimitError = mUpperLimit - uDotSliderAxis;
        mIsLowerLimitViolated = lowerLimitError <= 0.0f;
        mIsUpperLimitViolated = upperLimitError <= 0.0f;

        // Compute the cross products used in the Jacobians
        mR2CrossN1 = new Vector3(mR2).cross(mN1);
        mR2CrossN2 = new Vector3(mR2).cross(mN2);
        mR2CrossSliderAxis = new Vector3(mR2).cross(mSliderAxisWorld);
        Vector3 r1PlusU = new Vector3(mR1).add(u);
        mR1PlusUCrossN1 = new Vector3(r1PlusU).cross(mN1);
        mR1PlusUCrossN2 = new Vector3(r1PlusU).cross(mN2);
        mR1PlusUCrossSliderAxis = new Vector3(r1PlusU).cross(mSliderAxisWorld);

        /**
         * --------------- Translation Constraints ---------------
         */
        // Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
        // constraints (2x2 matrix)
        float sumInverseMass = 0.0f;
        Vector3 I1R1PlusUCrossN1 = new Vector3();
        Vector3 I1R1PlusUCrossN2 = new Vector3();
        Vector3 I2R2CrossN1 = new Vector3();
        Vector3 I2R2CrossN2 = new Vector3();
        if (mBody1.isMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1 = Matrix3x3.operatorMultiply(mI1, mR1PlusUCrossN1);
            I1R1PlusUCrossN2 = Matrix3x3.operatorMultiply(mI1, mR1PlusUCrossN2);
        }
        if (mBody2.isMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1 = Matrix3x3.operatorMultiply(mI2, mR2CrossN1);
            I2R2CrossN2 = Matrix3x3.operatorMultiply(mI2, mR2CrossN2);
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
        Vector2 lambdaTranslation = Matrix2x2.operatorMultiply(
                mInverseMassMatrixTranslationConstraint, new Vector2(translationError).invert());

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody1 = new Vector3(
                    new Vector3(new Vector3(mN1).invert()).multiply(lambdaTranslation.getX())).subtract(
                            new Vector3(mN2).multiply(lambdaTranslation.getY()));
            Vector3 angularImpulseBody1 = new Vector3(
                    new Vector3(new Vector3(mR1PlusUCrossN1).invert()).multiply(lambdaTranslation.getX())).subtract(
                            new Vector3(mR1PlusUCrossN2).multiply(lambdaTranslation.getY()));

            // Apply the impulse to the body
            Vector3 v1 = new Vector3(linearImpulseBody1).multiply(inverseMassBody1);
            Vector3 w1 = Matrix3x3.operatorMultiply(mI1, angularImpulseBody1);

            // Update the body position/orientation
            x1.add(v1);
            q1.add(new Quaternion(w1, 0.0f).multiply(q1).multiply(0.5f));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            Vector3 linearImpulseBody2 = new Vector3(
                    new Vector3(new Vector3(mN1).invert()).multiply(lambdaTranslation.getX())).add(
                            new Vector3(mN2).multiply(lambdaTranslation.getY()));
            Vector3 angularImpulseBody2 = new Vector3(
                    new Vector3(new Vector3(mR2CrossN1).invert()).multiply(lambdaTranslation.getX())).add(
                            new Vector3(mR2CrossN2).multiply(lambdaTranslation.getY()));

            // Apply the impulse to the body
            Vector3 v2 = new Vector3(linearImpulseBody2).multiply(inverseMassBody2);
            Vector3 w2 = Matrix3x3.operatorMultiply(mI2, angularImpulseBody2);

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
        mInverseMassMatrixRotationConstraint.setToZero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.operatorAddEqual(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.operatorAddEqual(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
        }

        // Compute the position error for the 3 rotation constraints
        Quaternion currentOrientationDifference = new Quaternion(q2).multiply(new Quaternion(q1).inverse());
        currentOrientationDifference.normalize();
        Quaternion qError = new Quaternion(currentOrientationDifference).multiply(mInitOrientationDifferenceInv);
        Vector3 qErrorV = new Vector3();
        qError.getVectorV(qErrorV);
        Vector3 errorRotation = qErrorV.multiply(2.0f);

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        Vector3 lambdaRotation = Matrix3x3.operatorMultiply(
                mInverseMassMatrixRotationConstraint, new Vector3(errorRotation).invert());

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody1 = new Vector3(lambdaRotation).invert();

            // Apply the impulse to the body
            Vector3 w1 = Matrix3x3.operatorMultiply(mI1, angularImpulseBody1);

            // Update the body position/orientation
            q1.add(new Quaternion(w1, 0.0f).multiply(q1).multiply(0.5f));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            Vector3 angularImpulseBody2 = lambdaRotation;

            // Apply the impulse to the body
            Vector3 w2 = Matrix3x3.operatorMultiply(mI2, angularImpulseBody2);

            // Update the body position/orientation
            q2.add(new Quaternion(w2, 0.0f).multiply(q2).multiply(0.5f));
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
                    mInverseMassMatrixLimit += mBody1.getMassInverse() + mR1PlusUCrossSliderAxis.dot(Matrix3x3.operatorMultiply(mI1, mR1PlusUCrossSliderAxis));
                }
                if (mBody2.isMotionEnabled()) {
                    mInverseMassMatrixLimit += mBody2.getMassInverse() + mR2CrossSliderAxis.dot(Matrix3x3.operatorMultiply(mI2, mR2CrossSliderAxis));
                }
                mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0f) ? 1.0f / mInverseMassMatrixLimit : 0.0f;
            }

            // If the lower limit is violated
            if (mIsLowerLimitViolated) {

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                float lambdaLowerLimit = mInverseMassMatrixLimit * (-lowerLimitError);

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    Vector3 linearImpulseBody1 = new Vector3(mSliderAxisWorld).multiply(-lambdaLowerLimit);
                    Vector3 angularImpulseBody1 = new Vector3(mR1PlusUCrossSliderAxis).multiply(-lambdaLowerLimit);

                    // Apply the impulse to the body
                    Vector3 v1 = new Vector3(linearImpulseBody1).multiply(inverseMassBody1);
                    Vector3 w1 = Matrix3x3.operatorMultiply(mI1, angularImpulseBody1);

                    // Update the body position/orientation
                    x1.add(v1);
                    q1.add(new Quaternion(w1, 0.0f).multiply(q1).multiply(0.5f));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    Vector3 linearImpulseBody2 = new Vector3(mSliderAxisWorld).multiply(lambdaLowerLimit);
                    Vector3 angularImpulseBody2 = new Vector3(mR2CrossSliderAxis).multiply(lambdaLowerLimit);

                    // Apply the impulse to the body
                    Vector3 v2 = new Vector3(linearImpulseBody2).multiply(inverseMassBody2);
                    Vector3 w2 = Matrix3x3.operatorMultiply(mI2, angularImpulseBody2);

                    // Update the body position/orientation
                    x2.add(v2);
                    q2.add(new Quaternion(w2, 0.0f).multiply(q2).multiply(0.5f));
                    q2.normalize();
                }
            }

            // If the upper limit is violated
            if (mIsUpperLimitViolated) {

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                float lambdaUpperLimit = mInverseMassMatrixLimit * (-upperLimitError);

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    Vector3 linearImpulseBody1 = new Vector3(mSliderAxisWorld).multiply(lambdaUpperLimit);
                    Vector3 angularImpulseBody1 = new Vector3(mR1PlusUCrossSliderAxis).multiply(lambdaUpperLimit);

                    // Apply the impulse to the body
                    Vector3 v1 = new Vector3(linearImpulseBody1).multiply(inverseMassBody1);
                    Vector3 w1 = Matrix3x3.operatorMultiply(mI1, angularImpulseBody1);

                    // Update the body position/orientation
                    x1.add(v1);
                    q1.add(new Quaternion(w1, 0.0f).multiply(q1).multiply(0.5f));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    Vector3 linearImpulseBody2 = new Vector3(mSliderAxisWorld).multiply(-lambdaUpperLimit);
                    Vector3 angularImpulseBody2 = new Vector3(mR2CrossSliderAxis).multiply(-lambdaUpperLimit);

                    // Apply the impulse to the body
                    Vector3 v2 = new Vector3(linearImpulseBody2).multiply(inverseMassBody2);
                    Vector3 w2 = Matrix3x3.operatorMultiply(mI2, angularImpulseBody2);

                    // Update the body position/orientation
                    x2.add(v2);
                    q2.add(new Quaternion(w2, 0.0f).multiply(q2).multiply(0.5f));
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
        Vector3 vB1 = new Vector3();
        q1.multiply(mLocalAnchorPointBody1, vB1);
        Vector3 anchorBody1 = new Vector3(x1).add(vB1);
        Vector3 vB2 = new Vector3();
        q2.multiply(mLocalAnchorPointBody2, vB2);
        Vector3 anchorBody2 = new Vector3(x2).add(vB2);

        // Compute the vector u (difference between anchor points)
        Vector3 u = new Vector3(anchorBody2).subtract(anchorBody1);

        // Compute the slider axis in world-space
        Vector3 sliderAxisWorld = new Vector3();
        q1.multiply(mSliderAxisBody1, sliderAxisWorld);
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
