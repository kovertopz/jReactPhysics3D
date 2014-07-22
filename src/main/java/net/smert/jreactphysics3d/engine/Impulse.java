package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * Represents an impulse that we can apply to bodies in the contact or constraint solver.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Impulse {

    /// Linear impulse applied to the first body
    public Vector3 linearImpulseBody1;

    /// Linear impulse applied to the second body
    public Vector3 linearImpulseBody2;

    /// Angular impulse applied to the first body
    public Vector3 angularImpulseBody1;

    /// Angular impulse applied to the second body
    public Vector3 angularImpulseBody2;

    /// Constructor
    public Impulse(Vector3 initLinearImpulseBody1, Vector3 initAngularImpulseBody1,
            Vector3 initLinearImpulseBody2, Vector3 initAngularImpulseBody2) {
        linearImpulseBody1 = initLinearImpulseBody1;
        angularImpulseBody1 = initAngularImpulseBody1;
        linearImpulseBody2 = initLinearImpulseBody2;
        angularImpulseBody2 = initAngularImpulseBody2;
    }

    /// Copy-constructor
    public Impulse(Impulse impulse) {
        linearImpulseBody1 = impulse.linearImpulseBody1;
        angularImpulseBody1 = impulse.angularImpulseBody1;
        linearImpulseBody2 = impulse.linearImpulseBody2;
        angularImpulseBody2 = impulse.angularImpulseBody2;
    }

}
