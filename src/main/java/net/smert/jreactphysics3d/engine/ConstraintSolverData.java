package net.smert.jreactphysics3d.engine;

import java.util.List;
import java.util.Map;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This structure contains data from the constraint solver that are used to solve each joint constraint.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ConstraintSolverData {

    // True if warm starting of the solver is active
    public boolean isWarmStartingActive;

    // Current time step of the simulation
    public float timeStep;

    // Reference to the bodies orientations
    public final List<Quaternion> orientations;

    // Reference to the bodies positions
    public final List<Vector3> positions;

    // Reference to the map that associates rigid body to their index
    // in the constrained velocities array
    public final Map<RigidBody, Integer> mapBodyToConstrainedVelocityIndex;

    // Array with the bodies angular velocities
    public Vector3[] angularVelocities;

    // Array with the bodies linear velocities
    public Vector3[] linearVelocities;

    // Constructor
    public ConstraintSolverData(List<Vector3> refPositions, List<Quaternion> refOrientations,
            Map<RigidBody, Integer> refMapBodyToConstrainedVelocityIndex) {
        orientations = refOrientations;
        positions = refPositions;
        mapBodyToConstrainedVelocityIndex = refMapBodyToConstrainedVelocityIndex;
        angularVelocities = null;
        linearVelocities = null;
    }

}
