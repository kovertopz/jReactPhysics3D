package net.smert.jreactphysics3d.examples.collisionshapes;

import java.io.IOException;
import net.smert.jreactphysics3d.collision.shapes.SphereShape;
import net.smert.jreactphysics3d.engine.DynamicsWorld;
import net.smert.jreactphysics3d.framework.Fw;
import net.smert.jreactphysics3d.framework.math.Transform4f;
import net.smert.jreactphysics3d.framework.math.Vector3f;
import net.smert.jreactphysics3d.framework.opengl.GL;
import net.smert.jreactphysics3d.framework.opengl.mesh.Mesh;
import net.smert.jreactphysics3d.framework.opengl.renderable.AbstractRenderable;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Sphere extends AbstractGameObjectShape {

    public Sphere(Vector3f position, float radius, float mass, DynamicsWorld dynamicsWorld) {

        // Scaling
        Vector3f size = new Vector3f(radius, radius, radius);
        Transform4f scaling = new Transform4f();
        scaling.getRotation().setDiagonal(size);
        setScalingTransform(scaling); // Attach to game object

        SphereShape collisionShape = new SphereShape(radius, 0.02f);
        createRigidBody(collisionShape, position, mass, dynamicsWorld);

        try {
            // Mesh
            Mesh mesh = GL.mf.createMesh();
            setMesh(mesh); // Attach mesh to game object
            Fw.graphics.loadMesh("primitives/uvsphere.obj", mesh);
            mesh.getSegment(0).setAllColors(0.8f, 0.8f, 0.8f, 1.0f);
            mesh.updateBooleansFromSegment();

            // Renderable
            AbstractRenderable renderable = Fw.graphics.createVertexBufferObjectRenderable();
            renderable.create(mesh);
            setRenderable(renderable); // Attach to game object
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }
    }

}
