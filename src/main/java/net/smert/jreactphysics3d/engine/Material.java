package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class contains the material properties of a rigid body that will be use for the dynamics simulation like the
 * friction coefficient or the bounciness of the rigid body.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Material {

    // Friction coefficient (positive value)
    private float mFrictionCoefficient;

    // Bounciness during collisions (between 0 and 1) where 1 is for a very bouncy body
    private float mBounciness;

    // Constructor
    public Material() {
        mFrictionCoefficient = Defaults.DEFAULT_FRICTION_COEFFICIENT;
        mBounciness = Defaults.DEFAULT_BOUNCINESS;
    }

    // Copy-constructor
    public Material(Material material) {
        mFrictionCoefficient = material.mFrictionCoefficient;
        mBounciness = material.mBounciness;
    }

    // Return the bounciness
    public float getBounciness() {
        return mBounciness;
    }

    // Set the bounciness.
    // The bounciness should be a value between 0 and 1. The value 1 is used for a
    // very bouncy body and zero is used for a body that is not bouncy at all.
    public void setBounciness(float bounciness) {
        assert (bounciness >= 0.0f && bounciness <= 1.0f);
        mBounciness = bounciness;
    }

    // Return the friction coefficient
    public float getFrictionCoefficient() {
        return mFrictionCoefficient;
    }

    // Set the friction coefficient.
    // The friction coefficient has to be a positive value. The value zero is used for no
    // friction at all.
    public void setFrictionCoefficient(float frictionCoefficient) {
        assert (frictionCoefficient >= 0.0f);
        mFrictionCoefficient = frictionCoefficient;
    }

    // Overloaded assignment operator
    public Material operatorEqual(Material material) {

        // Check for self-assignment
        if (this != material) {
            mFrictionCoefficient = material.mFrictionCoefficient;
            mBounciness = material.mBounciness;
        }

        // Return this material
        return this;
    }

}
