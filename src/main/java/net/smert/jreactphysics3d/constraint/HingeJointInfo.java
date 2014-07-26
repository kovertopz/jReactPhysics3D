package net.smert.jreactphysics3d.constraint;

import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This structure is used to gather the information needed to create a hinge joint. This structure will be used to
 * create the actual hinge joint.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class HingeJointInfo extends JointInfo {

    // Anchor point (in world-space coordinates)
    public Vector3 anchorPointWorldSpace;

    // Hinge rotation axis (in world-space coordinates)
    public Vector3 rotationAxisWorld;

    // True if the hinge joint limits are enabled
    public boolean isLimitEnabled;

    // True if the hinge joint motor is enabled
    public boolean isMotorEnabled;

    // Minimum allowed rotation angle (in radian) if limits are enabled.
    // The angle must be in the range [-2 pi, 0]
    public float minAngleLimit;

    // Maximum allowed rotation angle (in radian) if limits are enabled.
    // The angle must be in the range [0, 2 pi]
    public float maxAngleLimit;

    // Motor speed (in radian/second)
    public float motorSpeed;

    // Maximum motor torque (in Newtons   meters) that can be applied to reach
    // to desired motor speed
    public float maxMotorTorque;

    // Constructor
    public HingeJointInfo(JointType constraintType) {
        super(constraintType);
    }

    // Constructor without limits and without motor
    public HingeJointInfo(RigidBody rigidBody1, RigidBody rigidBody2,
            Vector3 initAnchorPointWorldSpace, Vector3 initRotationAxisWorld) {
        super(rigidBody1, rigidBody2, JointType.HINGEJOINT);
        anchorPointWorldSpace = initAnchorPointWorldSpace;
        rotationAxisWorld = initRotationAxisWorld;
        isLimitEnabled = true;
        isMotorEnabled = false;
        minAngleLimit = -1.0f;
        maxAngleLimit = 1.0f;
        motorSpeed = 0.0f;
        maxMotorTorque = 0.0f;
    }

    // Constructor with limits but without motor
    public HingeJointInfo(RigidBody rigidBody1, RigidBody rigidBody2,
            Vector3 initAnchorPointWorldSpace, Vector3 initRotationAxisWorld,
            float initMinAngleLimit, float initMaxAngleLimit) {
        super(rigidBody1, rigidBody2, JointType.HINGEJOINT);
        anchorPointWorldSpace = initAnchorPointWorldSpace;
        rotationAxisWorld = initRotationAxisWorld;
        isLimitEnabled = true;
        isMotorEnabled = false;
        minAngleLimit = initMinAngleLimit;
        maxAngleLimit = initMaxAngleLimit;
        motorSpeed = 0.0f;
        maxMotorTorque = 0.0f;
    }

    // Constructor with limits and motor
    public HingeJointInfo(RigidBody rigidBody1, RigidBody rigidBody2,
            Vector3 initAnchorPointWorldSpace, Vector3 initRotationAxisWorld,
            float initMinAngleLimit, float initMaxAngleLimit,
            float initMotorSpeed, float initMaxMotorTorque) {
        super(rigidBody1, rigidBody2, JointType.HINGEJOINT);
        anchorPointWorldSpace = initAnchorPointWorldSpace;
        rotationAxisWorld = initRotationAxisWorld;
        isLimitEnabled = true;
        isMotorEnabled = false;
        minAngleLimit = initMinAngleLimit;
        maxAngleLimit = initMaxAngleLimit;
        motorSpeed = initMotorSpeed;
        maxMotorTorque = initMaxMotorTorque;
    }

}
