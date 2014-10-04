package net.smert.jreactphysics3d.examples.collisionshapes;

import java.io.IOException;
import net.smert.jreactphysics3d.collision.shapes.BoxShape;
import net.smert.jreactphysics3d.engine.DynamicsWorld;
import net.smert.jreactphysics3d.framework.Fw;
import net.smert.jreactphysics3d.framework.math.Transform4f;
import net.smert.jreactphysics3d.framework.math.Vector3f;
import net.smert.jreactphysics3d.framework.opengl.GL;
import net.smert.jreactphysics3d.framework.opengl.mesh.Mesh;
import net.smert.jreactphysics3d.framework.opengl.renderable.AbstractRenderable;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Box extends AbstractGameObjectShape {

    public Box(Vector3f position, Vector3f size, float mass, DynamicsWorld dynamicsWorld) {

        // Scaling
        Vector3f halfSize = new Vector3f(size).multiply(0.5f);
        Transform4f scaling = new Transform4f();
        scaling.getRotation().setDiagonal(halfSize);
        setScalingTransform(scaling); // Attach to game object

        Vector3 extent = new Vector3(size.getX(), size.getY(), size.getZ()).multiply(0.5f);
        BoxShape collisionShape = new BoxShape(extent, 0.02f);
        createRigidBody(collisionShape, position, mass, dynamicsWorld);

        try {
            // Mesh
            Mesh mesh = GL.mf.createMesh();
            setMesh(mesh); // Attach mesh to game object
            Fw.graphics.loadMesh("primitives/cube.obj", mesh);
            mesh.setAllColors(0.3f, 0.3f, 0.3f, 1.0f);
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
