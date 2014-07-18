package net.smert.jreactphysics3d.constraint;

import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.configuration.JointsPositionCorrectionTechnique;

/**
 * This structure is used to gather the information needed to create a joint.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class JointInfo {

    /// First rigid body of the joint
    public RigidBody body1;

    /// Second rigid body of the joint
    public RigidBody body2;

    /// Type of the joint
    public JointType type;

    /// True if the two bodies of the joint are allowed to collide with each other
    public boolean isCollisionEnabled;

    /// Position correction technique used for the constraint (used for joints).
    /// By default, the BAUMGARTE technique is used
    public JointsPositionCorrectionTechnique positionCorrectionTechnique;

    /// Constructor
    public JointInfo(JointType constraintType) {
        body1 = null;
        body2 = null;
        type = constraintType;
        positionCorrectionTechnique = JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL;
        isCollisionEnabled = true;
    }

    /// Constructor
    public JointInfo(RigidBody rigidBody1, RigidBody rigidBody2, JointType constraintType) {
        body1 = rigidBody1;
        body2 = rigidBody2;
        type = constraintType;
        positionCorrectionTechnique = JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL;
        isCollisionEnabled = true;
    }

}
