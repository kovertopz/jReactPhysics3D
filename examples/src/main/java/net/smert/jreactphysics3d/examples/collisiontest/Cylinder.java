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

import java.io.IOException;
import net.smert.frameworkgl.Fw;
import net.smert.frameworkgl.math.Transform4f;
import net.smert.frameworkgl.math.Vector3f;
import net.smert.frameworkgl.opengl.GL;
import net.smert.frameworkgl.opengl.mesh.Mesh;
import net.smert.frameworkgl.opengl.renderable.AbstractRenderable;
import net.smert.jreactphysics3d.collision.shapes.CylinderShape;
import net.smert.jreactphysics3d.engine.DynamicsWorld;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Cylinder extends AbstractGameObjectShape {

    public Cylinder(Vector3f position, float height, float radius, float mass, DynamicsWorld dynamicsWorld) {

        // Scaling
        Vector3f size = new Vector3f(radius, height, radius);
        Transform4f scaling = new Transform4f();
        scaling.getRotation().setDiagonal(size);
        setScalingTransform(scaling); // Attach to game object

        CylinderShape collisionShape = new CylinderShape(radius, height, 0.02f);
        createRigidBody(collisionShape, position, mass, dynamicsWorld);

        try {
            // Mesh
            Mesh mesh = GL.meshFactory.createMesh();
            setMesh(mesh); // Attach mesh to game object
            Fw.graphics.loadMesh("primitives/cylinder.obj", mesh);
            mesh.setAllColors(0.8f, 0.8f, 0.8f, 1.0f);
            mesh.updateBooleansFromSegment();

            // Renderable
            AbstractRenderable renderable = GL.legacyRenderer.createVertexBufferObjectRenderable();
            renderable.create(mesh);
            setRenderable(renderable); // Attach to game object
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }
    }

}
