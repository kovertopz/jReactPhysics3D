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
package net.smert.jreactphysics3d.examples.collisiontest;

import net.smert.frameworkgl.gameobjects.GameObject;
import net.smert.frameworkgl.math.Vector3f;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.engine.DynamicsWorld;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class AbstractGameObjectShape extends GameObject {

    protected void createRigidBody(
            CollisionShape collisionShape, Vector3f position, float mass, DynamicsWorld dynamicsWorld) {

        setCollisionShape(collisionShape); // Attach to game object

        Matrix3x3 inertiaTensor = new Matrix3x3();
        collisionShape.computeLocalInertiaTensor(inertiaTensor, mass);

        Quaternion initOrientation = new Quaternion().identity();
        Vector3 initPosition = new Vector3(position.getX(), position.getY(), position.getZ());
        Transform transform = new Transform(initPosition, initOrientation);

        RigidBody rigidBody = dynamicsWorld.createRigidBody(transform, mass, inertiaTensor, collisionShape);
        setRigidBody(rigidBody); // Attach to game object
    }

    public void updateTransform() {

        // Get the interpolated transform of the rigid body
        RigidBody rigidBody = (RigidBody) getRigidBody();
        Transform transform = rigidBody.getInterpolatedTransform(new Transform());

        // Compute the transform used for rendering the box
        float[] glMatrix = new float[16];
        transform.getOpenGLMatrix(glMatrix);

        // Apply the scaling matrix to have the correct box dimensions
        getWorldTransform().fromOpenGLArray(glMatrix);
        getWorldTransform().multiply(getScalingTransform());
    }

}
