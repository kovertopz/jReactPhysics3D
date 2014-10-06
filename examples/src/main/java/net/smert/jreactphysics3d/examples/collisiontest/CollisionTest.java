/**
 * Copyright 2014 Jason Sorensen (sorensenj@smert.net)
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */
package net.smert.jreactphysics3d.examples.collisiontest;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.engine.ContactManifold;
import net.smert.jreactphysics3d.engine.DynamicsWorld;
import net.smert.jreactphysics3d.engine.Material;
import net.smert.jreactphysics3d.framework.Fw;
import net.smert.jreactphysics3d.framework.GameObject;
import net.smert.jreactphysics3d.framework.Screen;
import net.smert.jreactphysics3d.framework.helpers.Keyboard;
import net.smert.jreactphysics3d.framework.math.Vector3f;
import net.smert.jreactphysics3d.framework.opengl.GL;
import net.smert.jreactphysics3d.framework.opengl.camera.LegacyCamera;
import net.smert.jreactphysics3d.framework.opengl.camera.LegacyCameraController;
import net.smert.jreactphysics3d.framework.opengl.constants.GetString;
import net.smert.jreactphysics3d.framework.opengl.constants.Light;
import net.smert.jreactphysics3d.framework.utils.FpsTimer;
import net.smert.jreactphysics3d.framework.utils.MemoryUsage;
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
    private FloatBuffer transformWorldFloatBuffer;
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
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM1) == true) {
            cylinderMoveable.getRigidBody().applyTorque(new Vector3(0.5f, 0.0f, 0.0f));
        }
        if (Fw.input.isKeyDown(Keyboard.NUMPAD_NUM3) == true) {
            cylinderMoveable.getRigidBody().applyTorque(new Vector3(-0.5f, 0.0f, 0.0f));
        }
        if (direction.magnitudeSquared() > 0) {
            direction.normalize();
            cylinderMoveable.getRigidBody().applyForceToCenter(new Vector3(direction.getX(), direction.getY(), direction.getZ()));
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
        camera = new LegacyCamera();
        camera.setPosition(0.0f, 1.0f, 5.0f);
        cameraController = new LegacyCameraController(camera);

        // Time step and gravity for the physics simulation
        float timeStep = 1.0f / 60.0f;
        Vector3 gravity = new Vector3(0, -9.81f, 0);

        // Create dynamics world
        dynamicsWorld = new DynamicsWorld(gravity, timeStep);
        dynamicsWorld.setIsGratityEnabled(false);
        dynamicsWorld.start();

        // Float buffer for light and matrices
        lightFloatBuffer = GL.bufferHelper.createFloatBuffer(4);
        transformWorldFloatBuffer = GL.bufferHelper.createFloatBuffer(16);

        // Memory usage
        memoryUsage = new MemoryUsage();

        // Add spheres
        Material material;
        Box floor;
        floor = new Box(new Vector3f(), new Vector3f(40.0f, 0.5f, 40.0f), FLOOR_MASS, dynamicsWorld);
        floor.getRigidBody().setIsMotionEnabled(false);
        material = floor.getRigidBody().getMaterial();
        material.setBounciness(0.2f);
        gameObjectShapes.add(floor);
        cylinderMoveable = new Cylinder(new Vector3f(0.0f, 3.0f, 0.0f), CYLINDER_HEIGHT, CYLINDER_RADIUS, CYLINDER_MASS, dynamicsWorld);
        cylinderMoveable.getRigidBody().setIsMotionEnabled(true);
        material = cylinderMoveable.getRigidBody().getMaterial();
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
                Fw.graphics.render(gameObjectShape, transformWorldFloatBuffer);
            }

            // Render contact points
            GL.o1.disableDepthTest();
            for (GameObject visualContactPoint : visualContactPoints) {
                Fw.graphics.render(visualContactPoint, transformWorldFloatBuffer);
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
