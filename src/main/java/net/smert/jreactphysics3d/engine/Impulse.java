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
package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * Represents an impulse that we can apply to bodies in the contact or constraint solver.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Impulse {

    // Angular impulse applied to the first body
    private final Vector3 angularImpulseBody1;

    // Angular impulse applied to the second body
    private final Vector3 angularImpulseBody2;

    // Linear impulse applied to the first body
    private final Vector3 linearImpulseBody1;

    // Linear impulse applied to the second body
    private final Vector3 linearImpulseBody2;

    // Constructor
    public Impulse(Vector3 initLinearImpulseBody1, Vector3 initAngularImpulseBody1,
            Vector3 initLinearImpulseBody2, Vector3 initAngularImpulseBody2) {
        linearImpulseBody1 = new Vector3(initLinearImpulseBody1);
        angularImpulseBody1 = new Vector3(initAngularImpulseBody1);
        linearImpulseBody2 = new Vector3(initLinearImpulseBody2);
        angularImpulseBody2 = new Vector3(initAngularImpulseBody2);
    }

    // Copy-constructor
    public Impulse(Impulse impulse) {
        linearImpulseBody1 = new Vector3(impulse.linearImpulseBody1);
        angularImpulseBody1 = new Vector3(impulse.angularImpulseBody1);
        linearImpulseBody2 = new Vector3(impulse.linearImpulseBody2);
        angularImpulseBody2 = new Vector3(impulse.angularImpulseBody2);
    }

    public Vector3 getAngularImpulseBody1() {
        return angularImpulseBody1;
    }

    public Vector3 getAngularImpulseBody2() {
        return angularImpulseBody2;
    }

    public Vector3 getLinearImpulseBody1() {
        return linearImpulseBody1;
    }

    public Vector3 getLinearImpulseBody2() {
        return linearImpulseBody2;
    }

}
