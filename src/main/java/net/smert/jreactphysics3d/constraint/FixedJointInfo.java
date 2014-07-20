package net.smert.jreactphysics3d.constraint;

import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This structure is used to gather the information needed to create a fixed joint. This structure will be used to
 * create the actual fixed joint.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class FixedJointInfo extends JointInfo {

    /// Anchor point (in world-space coordinates)
    public Vector3 anchorPointWorldSpace;

    /// Constructor
    public FixedJointInfo(JointType constraintType) {
        super(constraintType);
    }

    /// Constructor
    public FixedJointInfo(RigidBody rigidBody1, RigidBody rigidBody2, Vector3 initAnchorPointWorldSpace) {
        super(rigidBody1, rigidBody2, JointType.FIXEDJOINT);

        anchorPointWorldSpace = initAnchorPointWorldSpace;
    }

}
