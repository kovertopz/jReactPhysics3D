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

import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.configuration.JointsPositionCorrectionTechnique;
import net.smert.jreactphysics3d.engine.ConstraintSolverData;

/**
 * This abstract class represents a joint between two bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class Joint {

    // Pointer to the first body of the joint
    protected RigidBody mBody1;

    // Pointer to the second body of the joint
    protected RigidBody mBody2;

    // Type of the joint
    protected JointType mType;

    // Body 1 index in the velocity array to solve the constraint
    protected int mIndexBody1;

    // Body 2 index in the velocity array to solve the constraint
    protected int mIndexBody2;

    // Position correction technique used for the constraint (used for joints)
    protected JointsPositionCorrectionTechnique mPositionCorrectionTechnique;

    // True if the two bodies of the constraint are allowed to collide with each other
    protected boolean mIsCollisionEnabled;

    // True if the joint has already been added into an island
    protected boolean mIsAlreadyInIsland;

    // Constructor
    public Joint(JointInfo jointInfo) {
        mBody1 = jointInfo.body1;
        mBody2 = jointInfo.body2;
        mType = jointInfo.type;
        mPositionCorrectionTechnique = jointInfo.positionCorrectionTechnique;
        mIsCollisionEnabled = jointInfo.isCollisionEnabled;
        mIsAlreadyInIsland = false;

        assert (mBody1 != null);
        assert (mBody2 != null);
    }

    // Return the reference to the body 1
    public RigidBody getBody1() {
        return mBody1;
    }

    // Return the reference to the body 2
    public RigidBody getBody2() {
        return mBody2;
    }

    // Return true if the joint is active
    public boolean isActive() {
        return (mBody1.isActive() && mBody2.isActive());
    }

    // Return the type of the joint
    public JointType getType() {
        return mType;
    }

    // Return true if the collision between the two bodies of the joint is enabled
    public boolean isCollisionEnabled() {
        return mIsCollisionEnabled;
    }

    // Return true if the joint has already been added into an island
    public boolean isAlreadyInIsland() {
        return mIsAlreadyInIsland;
    }

    public void setIsAlreadyInIsland(boolean isAlreadyInIsland) {
        mIsAlreadyInIsland = isAlreadyInIsland;
    }

    // Initialize before solving the joint
    public abstract void initBeforeSolve(ConstraintSolverData constraintSolverData);

    // Warm start the joint (apply the previous impulse at the beginning of the step)
    public abstract void warmstart(ConstraintSolverData constraintSolverData);

    // Solve the velocity constraint
    public abstract void solveVelocityConstraint(ConstraintSolverData constraintSolverData);

    // Solve the position constraint
    public abstract void solvePositionConstraint(ConstraintSolverData constraintSolverData);

}
