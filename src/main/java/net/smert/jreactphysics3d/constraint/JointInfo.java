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

/**
 * This structure is used to gather the information needed to create a joint.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class JointInfo {

    // First rigid body of the joint
    public RigidBody body1;

    // Second rigid body of the joint
    public RigidBody body2;

    // Type of the joint
    public JointType type;

    // True if the two bodies of the joint are allowed to collide with each other
    public boolean isCollisionEnabled;

    // Position correction technique used for the constraint (used for joints).
    // By default, the BAUMGARTE technique is used
    public JointsPositionCorrectionTechnique positionCorrectionTechnique;

    // Constructor
    public JointInfo(JointType constraintType) {
        body1 = null;
        body2 = null;
        type = constraintType;
        positionCorrectionTechnique = JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL;
        isCollisionEnabled = true;
    }

    // Constructor
    public JointInfo(RigidBody rigidBody1, RigidBody rigidBody2, JointType constraintType) {
        body1 = rigidBody1;
        body2 = rigidBody2;
        type = constraintType;
        positionCorrectionTechnique = JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL;
        isCollisionEnabled = true;
    }

}
