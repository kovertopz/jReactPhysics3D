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
package net.smert.jreactphysics3d.examples.collisionshapes;

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
import net.smert.jreactphysics3d.framework.math.MathHelper;
import net.smert.jreactphysics3d.framework.math.Vector3f;
import net.smert.jreactphysics3d.framework.opengl.GL;
import net.smert.jreactphysics3d.framework.opengl.camera.LegacyCamera;
import net.smert.jreactphysics3d.framework.opengl.camera.LegacyCameraController;
import net.smert.jreactphysics3d.framework.opengl.constants.GetString;
import net.smert.jreactphysics3d.framework.utils.FpsTimer;
import net.smert.jreactphysics3d.framework.utils.MemoryUsage;
import net.smert.jreactphysics3d.mathematics.Vector3;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CollisionShapes extends Screen {

    private final static Logger log = LoggerFactory.getLogger(CollisionShapes.class);
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
    private static final int MAX_BOXES = 5;
    private static final int MAX_CAPSULES = 5;
    private static final int MAX_CONES = 5;
    private static final int MAX_CYLINDERS = 5;
    private static final int MAX_SPHERES = 5;
    private static final Vector3f BOX_SIZE = new Vector3f(2.0f, 2.0f, 2.0f);
    private static final Vector3f FLOOR_SIZE = new Vector3f(50.0f, 0.5f, 50.0f);

    private DynamicsWorld dynamicsWorld;
    private FloatBuffer transformWorldFloatBuffer;
    private FpsTimer fpsTimer;
    private LegacyCamera camera;
    private LegacyCameraController cameraController;
    private final List<AbstractGameObjectShape> gameObjectShapes;
    private final List<VisualContactPoint> visualContactPoints;
    private MemoryUsage memoryUsage;

    public CollisionShapes(String[] args) {
        gameObjectShapes = new ArrayList<>();
        visualContactPoints = new ArrayList<>();
    }

    private void createBoxes() {

        // Create all the boxes of the scene
        for (int i = 0; i < MAX_BOXES; i++) {

            // Position
            float angle = i * 30.0f;
            float radius = 3.0f;
            Vector3f position = new Vector3f(
                    radius * MathHelper.Cos(angle),
                    60.0f + i * (BOX_SIZE.getY() + 0.8f),
                    radius * MathHelper.Sin(angle));

            // Create a box and a corresponding rigid body in the dynamics world
            Box box = new Box(position, BOX_SIZE, BOX_MASS, dynamicsWorld);

            // The box is a moving rigid body
            box.getRigidBody().setIsMotionEnabled(true);

            // Change the material properties of the rigid body
            Material material = box.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the box to the list of game objects in the scene
            gameObjectShapes.add(box);
        }
    }

    private void createCapsule() {

        // Create all the capsules of the scene
        for (int i = 0; i < MAX_CAPSULES; i++) {

            // Position
            float angle = i * 45.0f;
            float radius = 3.0f;
            Vector3f position = new Vector3f(
                    radius * MathHelper.Cos(angle),
                    15.0f + i * (CAPSULE_HEIGHT + 0.5f),
                    radius * MathHelper.Sin(angle));

            // Create a capsule and a corresponding rigid body in the dynamics world
            Capsule capsule = new Capsule(position, CAPSULE_HEIGHT, CAPSULE_RADIUS, CAPSULE_MASS, dynamicsWorld);

            // The cylinder is a moving rigid body
            capsule.getRigidBody().setIsMotionEnabled(true);

            // Change the material properties of the rigid body
            Material material = capsule.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the capsule to the list of game objects in the scene
            gameObjectShapes.add(capsule);
        }
    }

    private void createCones() {

        // Create all the cones of the scene
        for (int i = 0; i < MAX_CONES; i++) {

            // Position
            float angle = i * 50.0f;
            float radius = 3.0f;
            Vector3f position = new Vector3f(
                    radius * MathHelper.Cos(angle),
                    35.0f + i * (CONE_HEIGHT + 0.3f),
                    radius * MathHelper.Sin(angle));

            // Create a cone and a corresponding rigid body in the dynamics world
            Cone cone = new Cone(position, CONE_HEIGHT, CONE_RADIUS, CONE_MASS, dynamicsWorld);

            // The cone is a moving rigid body
            cone.getRigidBody().setIsMotionEnabled(true);

            // Change the material properties of the rigid body
            Material material = cone.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the cone to the list of game objects in the scene
            gameObjectShapes.add(cone);
        }
    }

    private void createCylinder() {

        // Create all the cylinders of the scene
        for (int i = 0; i < MAX_CYLINDERS; i++) {

            // Position
            float angle = i * 35.0f;
            float radius = 3.0f;
            Vector3f position = new Vector3f(
                    radius * MathHelper.Cos(angle),
                    25.0f + i * (CYLINDER_HEIGHT + 0.3f),
                    radius * MathHelper.Sin(angle));

            // Create a cylinder and a corresponding rigid body in the dynamics world
            Cylinder cylinder = new Cylinder(position, CYLINDER_HEIGHT, CYLINDER_RADIUS, CYLINDER_MASS, dynamicsWorld);

            // The cylinder is a moving rigid body
            cylinder.getRigidBody().setIsMotionEnabled(true);

            // Change the material properties of the rigid body
            Material material = cylinder.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the cylinder to the list of game objects in the scene
            gameObjectShapes.add(cylinder);
        }
    }

    private void createFloor() {

        // Create the floor
        Vector3f floorPosition = new Vector3f();
        Box floor = new Box(floorPosition, FLOOR_SIZE, FLOOR_MASS, dynamicsWorld);

        // The floor must be a non-moving rigid body
        floor.getRigidBody().setIsMotionEnabled(false);

        // Change the material properties of the rigid body
        Material material = floor.getRigidBody().getMaterial();
        material.setBounciness(0.2f);

        // Add the floor to the list of game objects in the scene
        gameObjectShapes.add(floor);
    }

    private void createSpheres() {

        // Create all the spheres of the scene
        for (int i = 0; i < MAX_SPHERES; i++) {

            // Position
            float angle = i * 35.0f;
            float radius = 3.0f;
            Vector3f position = new Vector3f(
                    radius * MathHelper.Cos(angle),
                    50.0f + i * (SPHERE_RADIUS + 0.8f),
                    radius * MathHelper.Sin(angle));

            // Create a sphere and a corresponding rigid body in the dynamics world
            Sphere sphere = new Sphere(position, SPHERE_RADIUS, SPHERE_MASS, dynamicsWorld);

            // The sphere is a moving rigid body
            sphere.getRigidBody().setIsMotionEnabled(true);

            // Change the material properties of the rigid body
            Material material = sphere.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the sphere to the list of game objects in the scene
            gameObjectShapes.add(sphere);
        }
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
        cameraController.update();
    }

    @Override
    public void destroy() {
        for (GameObject gameObjectShape : gameObjectShapes) {
            gameObjectShape.destroy();
        }
        Fw.input.removeInputProcessor(cameraController);
        Fw.input.releaseMouseCursor();
    }

    @Override
    public void init() {

        // Create timer
        fpsTimer = new FpsTimer();

        // Setup camera and controller
        camera = new LegacyCamera();
        camera.setPosition(0.0f, 6.0f, 25.0f);
        cameraController = new LegacyCameraController(camera);

        // Time step and gravity for the physics simulation
        float timeStep = 1.0f / 60.0f;
        Vector3 gravity = new Vector3(0, -9.81f, 0);

        // Create dynamics world
        dynamicsWorld = new DynamicsWorld(gravity, timeStep);
        dynamicsWorld.start();

        // Float buffer for matrices
        transformWorldFloatBuffer = GL.bufferHelper.createFloatBuffer(16);

        // Memory usage
        memoryUsage = new MemoryUsage();

        // Create game objects
        createBoxes();
        createCapsule();
        createCones();
        createCylinder();
        createFloor();
        createSpheres();

        GL.o1.enableCulling();
        GL.o1.cullBackFaces();
        GL.o1.enableDepthTest();
        GL.o1.setDepthFuncLess();
        GL.o1.enableDepthMask();
        GL.o1.setClearDepth(1.0f);
        GL.o1.setSmoothLighting(true);
        GL.o1.clear();

        GL.o1.setProjectionPerspective(
                70.0f,
                (float) Fw.config.getCurrentWidth() / (float) Fw.config.getCurrentHeight(),
                0.05f, 512.0f);
        GL.o1.setModelViewIdentity();

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
