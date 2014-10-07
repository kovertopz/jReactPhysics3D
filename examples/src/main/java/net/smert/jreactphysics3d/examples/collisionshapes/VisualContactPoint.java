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
            staticMesh.setAllColors(1.0f, 1.0f, 0.0f, 1.0f);
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
