package net.smert.jreactphysics3d.examples.collisionshapes;

import java.io.IOException;
import net.smert.jreactphysics3d.framework.Fw;
import net.smert.jreactphysics3d.framework.GameObject;
import net.smert.jreactphysics3d.framework.math.Transform4f;
import net.smert.jreactphysics3d.framework.math.Vector3f;
import net.smert.jreactphysics3d.framework.opengl.GL;
import net.smert.jreactphysics3d.framework.opengl.mesh.Mesh;
import net.smert.jreactphysics3d.framework.opengl.renderable.AbstractRenderable;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class VisualContactPoint extends GameObject {

    private static final float VISUAL_CONTACT_POINT_RADIUS = 0.1f;

    private static boolean initialized;
    private static Mesh staticMesh;
    private static AbstractRenderable staticRenderable;

    public VisualContactPoint(Vector3f position) {

        // Scaling
        Vector3f size = new Vector3f(VISUAL_CONTACT_POINT_RADIUS, VISUAL_CONTACT_POINT_RADIUS, VISUAL_CONTACT_POINT_RADIUS);
        Transform4f scaling = new Transform4f();
        scaling.getRotation().setDiagonal(size);
        setScalingTransform(scaling); // Attach to game object

        getWorldTransform().getPosition().set(position);
        getWorldTransform().multiply(getScalingTransform());

        CreateStaticData();
        setMesh(staticMesh); // Attach mesh to game object
        setRenderable(staticRenderable); // Attach to game object
    }

    public static void CreateStaticData() {
        if (initialized) {
            return;
        }

        try {
            // Mesh
            staticMesh = GL.mf.createMesh();
            Fw.graphics.loadMesh("primitives/cone.obj", staticMesh);
            staticMesh.getSegment(0).setAllColors(1.0f, 1.0f, 0.0f, 1.0f);
            staticMesh.updateBooleansFromSegment();

            // Renderable
            staticRenderable = Fw.graphics.createVertexBufferObjectRenderable();
            staticRenderable.create(staticMesh);
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }

        // Only do this once
        initialized = true;
    }

    public static void Destroy() {
        if (!initialized) {
            return;
        }
        staticRenderable.destroy();
        staticRenderable = null;
        initialized = false;
    }

}
