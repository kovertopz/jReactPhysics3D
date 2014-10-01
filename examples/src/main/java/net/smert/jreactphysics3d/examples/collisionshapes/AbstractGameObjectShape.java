package net.smert.jreactphysics3d.examples.collisionshapes;

import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.engine.DynamicsWorld;
import net.smert.jreactphysics3d.framework.GameObject;
import net.smert.jreactphysics3d.framework.math.Vector3f;
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

        setCollisionBodyShape(collisionShape); // Attach to game object

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
        Transform transform = getRigidBody().getInterpolatedTransform(new Transform());

        // Compute the transform used for rendering the box
        float[] glMatrix = new float[16];
        transform.getOpenGLMatrix(glMatrix);

        // Apply the scaling matrix to have the correct box dimensions
        getWorldTransform().fromOpenGLArray(glMatrix);
        getWorldTransform().multiply(getScalingTransform());
    }

}
