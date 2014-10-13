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

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import net.smert.frameworkgl.Fw;
import net.smert.frameworkgl.gameobjects.GameObject;
import net.smert.frameworkgl.Screen;
import net.smert.frameworkgl.helpers.Keyboard;
import net.smert.frameworkgl.math.Vector3f;
import net.smert.frameworkgl.opengl.GL;
import net.smert.frameworkgl.opengl.camera.LegacyCamera;
import net.smert.frameworkgl.opengl.camera.LegacyCameraController;
import net.smert.frameworkgl.opengl.constants.GetString;
import net.smert.frameworkgl.opengl.constants.Light;
import net.smert.frameworkgl.utils.FpsTimer;
import net.smert.frameworkgl.utils.MemoryUsage;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.engine.ContactManifold;
import net.smert.jreactphysics3d.engine.DynamicsWorld;
import net.smert.jreactphysics3d.engine.Material;
import net.smert.jreactphysics3d.mathematics.Vector3;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CollisionTest extends Screen {

    private final static Logger log = LoggerFactory.getLogger(CollisionTest.class);
    private static final float BOX_MASS = 1.0f;
    private static final float CAPSULE_HEIGHT = 1.0f;
    private static final float CAPSULE_MASS = 1.0f;
    private static final float CAPSULE_RADIUS = 1.0f;
    private static final float CONE_HEIGHT = 3.0f;
    private static final float CONE_MASS = 1.0f;
    private static final float CONE_RADIUS = 2.0f;
    private static final float CYLINDER_HEIGHT = 5.0f;
    private static final float CYLINDER_MASS = 1.0f;
    private static final float CYLINDER_RADIUS = 1.0f;
    private static final float FLOOR_MASS = 100.0f;
    private static final float SPHERE_MASS = 1.5f;
    private static final float SPHERE_RADIUS = 1.0f;
    private static final Vector3f BOX_SIZE = new Vector3f(2.0f, 2.0f, 2.0f);

    private DynamicsWorld dynamicsWorld;
    private FloatBuffer lightFloatBuffer;
    private FpsTimer fpsTimer;
    private LegacyCamera camera;
    private LegacyCameraController cameraController;
    private final List<AbstractGameObjectShape> gameObjectShapes;
    private final List<VisualContactPoint> visualContactPoints;
    private MemoryUsage memoryUsage;

    public CollisionTest(String[] args) {
        gameObjectShapes = new ArrayList<>();
        visualContactPoints = new ArrayList<>();
    }

    private void generateContactPoints() {
        visualContactPoints.clear();
        List<ContactManifold> contactManifolds = dynamicsWorld.getContactManifolds();
        for (ContactManifold contactManifold : contactManifolds) {
            for (int i = 0; i < contactManifold.getNumContactPoints(); i++) {
                ContactPoint point = contactManifold.getContactPoint(i);
                Vector3 pos = point.getWorldPointOnBody1();
                Vector3f position = new Vector3f(pos.getX(), pos.getY(), pos.getZ());
                VisualContactPoint visualPoint = new VisualContactPoint(position);
                visualContactPoints.add(visualPoint);
            }
        }
    }

    private void handleInput() {
        if (Fw.input.isKeyDown(Keyboard.ESCAPE) == true) {
            Fw.app.stopRunning();
        }
        Vector3f direction = new Vector3f();
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM8) == true) {
            direction.setY(1.0f);
        }
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM2) == true) {
            direction.setY(-1.0f);
        }
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM4) == true) {
            direction.setX(-1.0f);
        }
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM6) == true) {
            direction.setX(1.0f);
        }
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM7) == true) {
            direction.setZ(-1.0f);
        }
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM9) == true) {
            direction.setZ(1.0f);
        }
        RigidBody rigidBody = (RigidBody) cylinderMoveable.getRigidBody();
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM1) == true) {
            rigidBody.applyTorque(new Vector3(0.5f, 0.0f, 0.0f));
        }
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM3) == true) {
            rigidBody.applyTorque(new Vector3(-0.5f, 0.0f, 0.0f));
        }
        if (direction.magnitudeSquared() > 0) {
            direction.normalize();
            rigidBody.applyForceToCenter(new Vector3(direction.getX(), direction.getY(), direction.getZ()));
        }
        cameraController.update();
    }

    @Override
    public void destroy() {
        for (GameObject gameObjectShape : gameObjectShapes) {
            gameObjectShape.destroy();
        }
        VisualContactPoint.Destroy();
        Fw.input.removeInputProcessor(cameraController);
        Fw.input.releaseMouseCursor();
    }

    private Cylinder cylinderMoveable;

    @Override
    public void init() {

        // Create timer
        fpsTimer = new FpsTimer();

        // Setup camera and controller
        camera = GL.cameraFactory.createLegacyCamera();
        camera.setPosition(0.0f, 1.0f, 5.0f);
        cameraController = GL.cameraFactory.createLegacyCameraController();
        cameraController.setCamera(camera);

        // Time step and gravity for the physics simulation
        float timeStep = 1.0f / 60.0f;
        Vector3 gravity = new Vector3(0, -9.81f, 0);

        // Create dynamics world
        dynamicsWorld = new DynamicsWorld(gravity, timeStep);
        dynamicsWorld.setIsGratityEnabled(false);
        dynamicsWorld.start();

        // Float buffer for light and matrices
        lightFloatBuffer = GL.bufferHelper.createFloatBuffer(4);

        // Memory usage
        memoryUsage = new MemoryUsage();

        // Add spheres
        Material material;
        RigidBody rigidBody;
        Box floor;
        floor = new Box(new Vector3f(), new Vector3f(40.0f, 0.5f, 40.0f), FLOOR_MASS, dynamicsWorld);
        rigidBody = (RigidBody) floor.getRigidBody();
        rigidBody.setIsMotionEnabled(false);
        material = rigidBody.getMaterial();
        material.setBounciness(0.2f);
        gameObjectShapes.add(floor);
        cylinderMoveable = new Cylinder(new Vector3f(0.0f, 3.0f, 0.0f), CYLINDER_HEIGHT, CYLINDER_RADIUS, CYLINDER_MASS, dynamicsWorld);
        rigidBody = (RigidBody) cylinderMoveable.getRigidBody();
        rigidBody.setIsMotionEnabled(true);
        material = rigidBody.getMaterial();
        material.setBounciness(0.2f);
        gameObjectShapes.add(cylinderMoveable);

        GL.o1.enableCulling();
        GL.o1.cullBackFaces();
        GL.o1.enableDepthTest();
        GL.o1.setDepthFuncLess();
        GL.o1.enableDepthMask();
        GL.o1.setClearDepth(1.0f);
        GL.o1.enableColorMaterial();
        GL.o1.enableLight0();
        GL.o1.enableLighting();
        GL.o1.setSmoothLighting(true);
        GL.o1.enableNormalize();
        GL.o1.clear();

        GL.o1.setProjectionPerspective(
                70.0f,
                (float) Fw.config.getCurrentWidth() / (float) Fw.config.getCurrentHeight(),
                0.05f, 512.0f);
        GL.o1.setModelViewIdentity();

        // Light position
        lightFloatBuffer.put(0.0f);
        lightFloatBuffer.put(15.0f);
        lightFloatBuffer.put(0.0f);
        lightFloatBuffer.put(1.0f);
        lightFloatBuffer.flip();

        log.info("OpenGL version: " + GL.o1.getString(GetString.VERSION));

        Fw.input.addInputProcessor(cameraController);
        Fw.input.grabMouseCursor();
    }

    @Override
    public void pause() {
    }

    @Override
    public void render() {
        fpsTimer.update();
        memoryUsage.update();

        if (Fw.timer.isGameTick()) {
            // Do nothing
        }

        if (Fw.timer.isRenderTick()) {
            handleInput();

            // Take a simulation step
            dynamicsWorld.update();

            // Generate the new visual contact points
            generateContactPoints();

            // Clear screen and reset modelview matrix
            GL.o1.clear();
            GL.o1.setModelViewIdentity();

            camera.updateOpenGL();

            GL.o1.light(Light.LIGHT0, Light.POSITION, lightFloatBuffer);

            // Render directly
            for (AbstractGameObjectShape gameObjectShape : gameObjectShapes) {
                gameObjectShape.updateTransform();
                GL.legacyRenderer.render(gameObjectShape);
            }

            // Render contact points
            GL.o1.disableDepthTest();
            for (GameObject visualContactPoint : visualContactPoints) {
                GL.legacyRenderer.render(visualContactPoint);
            }
            GL.o1.enableDepthTest();
        }
    }

    @Override
    public void resize(int width, int height) {
        GL.o1.setViewport(0, 0, width, height);
    }

    @Override
    public void resume() {
    }

}
