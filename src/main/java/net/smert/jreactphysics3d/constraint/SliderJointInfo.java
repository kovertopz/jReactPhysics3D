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
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This structure is used to gather the information needed to create a slider joint. This structure will be used to
 * create the actual slider joint.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class SliderJointInfo extends JointInfo {

    // Anchor point (in world-space coordinates)
    public Vector3 anchorPointWorldSpace;

    // Slider axis (in world-space coordinates)
    public Vector3 sliderAxisWorldSpace;

    // True if the slider limits are enabled
    public boolean isLimitEnabled;

    // True if the slider motor is enabled
    public boolean isMotorEnabled;

    // Mininum allowed translation if limits are enabled
    public float minTranslationLimit;

    // Maximum allowed translation if limits are enabled
    public float maxTranslationLimit;

    // Motor speed
    public float motorSpeed;

    // Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
    public float maxMotorForce;

    // Constructor
    public SliderJointInfo(JointType constraintType) {
        super(constraintType);
    }

    // Constructor without limits and without motor
    public SliderJointInfo(RigidBody rigidBody1, RigidBody rigidBody2,
            Vector3 initAnchorPointWorldSpace, Vector3 initSliderAxisWorldSpace) {
        super(rigidBody1, rigidBody2, JointType.SLIDERJOINT);
        anchorPointWorldSpace = initAnchorPointWorldSpace;
        sliderAxisWorldSpace = initSliderAxisWorldSpace;
        isLimitEnabled = false;
        isMotorEnabled = false;
        minTranslationLimit = -1.0f;
        maxTranslationLimit = 1.0f;
        motorSpeed = 0.0f;
        maxMotorForce = 0.0f;
    }

    // Constructor with limits and no motor
    public SliderJointInfo(RigidBody rigidBody1, RigidBody rigidBody2,
            Vector3 initAnchorPointWorldSpace, Vector3 initSliderAxisWorldSpace,
            float initMinTranslationLimit, float initMaxTranslationLimit) {
        super(rigidBody1, rigidBody2, JointType.SLIDERJOINT);
        anchorPointWorldSpace = initAnchorPointWorldSpace;
        sliderAxisWorldSpace = initSliderAxisWorldSpace;
        isLimitEnabled = true;
        isMotorEnabled = false;
        minTranslationLimit = initMinTranslationLimit;
        maxTranslationLimit = initMaxTranslationLimit;
        motorSpeed = 0.0f;
        maxMotorForce = 0.0f;
    }

    // Constructor with limits and motor
    public SliderJointInfo(RigidBody rigidBody1, RigidBody rigidBody2,
            Vector3 initAnchorPointWorldSpace, Vector3 initSliderAxisWorldSpace,
            float initMinTranslationLimit, float initMaxTranslationLimit,
            float initMotorSpeed, float initMaxMotorForce) {
        super(rigidBody1, rigidBody2, JointType.SLIDERJOINT);
        anchorPointWorldSpace = initAnchorPointWorldSpace;
        sliderAxisWorldSpace = initSliderAxisWorldSpace;
        isLimitEnabled = true;
        isMotorEnabled = true;
        minTranslationLimit = initMinTranslationLimit;
        maxTranslationLimit = initMaxTranslationLimit;
        motorSpeed = initMotorSpeed;
        maxMotorForce = initMaxMotorForce;
    }

}
