package net.smert.jreactphysics3d.collisionshapes;

import java.util.ArrayList;
import java.util.List;
import net.smert.jreactphysics3d.common.Box;
import net.smert.jreactphysics3d.common.Capsule;
import net.smert.jreactphysics3d.common.Cone;
import net.smert.jreactphysics3d.common.ConvexMesh;
import net.smert.jreactphysics3d.common.Cylinder;
import net.smert.jreactphysics3d.common.Sphere;
import net.smert.jreactphysics3d.common.VisualContactPoint;
import net.smert.jreactphysics3d.common.openglframework.Camera;
import net.smert.jreactphysics3d.common.openglframework.GlutViewer;
import net.smert.jreactphysics3d.common.openglframework.Light;
import net.smert.jreactphysics3d.common.openglframework.Shader;
import net.smert.jreactphysics3d.common.openglframework.maths.Color;
import net.smert.jreactphysics3d.common.openglframework.maths.Matrix4;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;
import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.engine.ContactManifold;
import net.smert.jreactphysics3d.engine.DynamicsWorld;
import net.smert.jreactphysics3d.engine.Material;
import org.lwjgl.opengl.GL11;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Scene {

    private static final int NB_BOXES = 3;
    private static final int NB_SPHERES = 1;
    private static final int NB_CONES = 3;
    private static final int NB_CYLINDERS = 3;
    private static final int NB_CAPSULES = 1;
    private static final int NB_MESHES = 0;
    private Vector3 BOX_SIZE = new Vector3(2, 2, 2);
    private static final float SPHERE_RADIUS = 1.5f;
    private static final float CONE_RADIUS = 2.0f;
    private static final float CONE_HEIGHT = 3.0f;
    private static final float CYLINDER_RADIUS = 1.0f;
    private static final float CYLINDER_HEIGHT = 5.0f;
    private static final float CAPSULE_RADIUS = 1.0f;
    private static final float CAPSULE_HEIGHT = 1.0f;
    private Vector3 FLOOR_SIZE = new Vector3(20, 0.5f, 20);        // Floor dimensions in meters
    private static final float BOX_MASS = 1.0f;
    private static final float CONE_MASS = 1.0f;
    private static final float CYLINDER_MASS = 1.0f;
    private static final float CAPSULE_MASS = 1.0f;
    private static final float MESH_MASS = 1.0f;
    private static final float FLOOR_MASS = 100.0f;                            // Floor mass in kilograms

    /// Pointer to the viewer
    private GlutViewer mViewer;

    /// Light 0
    private Light mLight0;

    /// Phong shader
    private Shader mPhongShader;

    /// All the spheres of the scene
    private List<Box> mBoxes = new ArrayList<>();
    private List<Sphere> mSpheres = new ArrayList<>();
    private List<Cone> mCones = new ArrayList<>();
    private List<Cylinder> mCylinders = new ArrayList<>();
    private List<Capsule> mCapsules = new ArrayList<>();

    /// All the convex meshes of the scene
    private List<ConvexMesh> mConvexMeshes = new ArrayList<>();

    /// All the visual contact points
    private List<VisualContactPoint> mContactPoints = new ArrayList<>();

    /// Box for the floor
    private Box mFloor;

    /// Dynamics world used for the physics simulation
    private DynamicsWorld mDynamicsWorld;

    /// True if the physics simulation is running
    private boolean mIsRunning;

    // Constructor
    public Scene(GlutViewer viewer) {

        mViewer = viewer;
        mLight0 = new Light(0);
        mPhongShader = new Shader("shaders/phong.vert", "shaders/phong.frag");
        mIsRunning = false;

        // Move the light 0
        mLight0.translateWorld(new Vector3(50, 50, 50));

        // Compute the radius and the center of the scene
        float radiusScene = 10.0f;
        Vector3 center = new Vector3(0, 5, 0);

        // Set the center of the scene
        mViewer.setScenePosition(center, radiusScene);

        // Gravity vector in the dynamics world
        net.smert.jreactphysics3d.mathematics.Vector3 gravity = new net.smert.jreactphysics3d.mathematics.Vector3(0, -9.81f, 0);

        // Time step for the physics simulation
        float timeStep = 1.0f / 60.0f;

        // Create the dynamics world for the physics simulation
        mDynamicsWorld = new DynamicsWorld(gravity, timeStep);

        // Set the number of iterations of the constraint solver
        mDynamicsWorld.setNbIterationsVelocitySolver(15);

        // Create the static data for the visual contact points
        VisualContactPoint.createStaticData();

        float radius = 3.0f;

        // Create all the boxes of the scene
        for (int i = 0; i < NB_BOXES; i++) {

            // Position
            float angle = i * 30.0f;
            Vector3 position = new Vector3(radius * (float) Math.cos(angle),
                    60 + i * (BOX_SIZE.y + 0.8f),
                    radius * (float) Math.sin(angle));

            // Create a sphere and a corresponding rigid in the dynamics world
            Box box = new Box(BOX_SIZE, position, BOX_MASS, mDynamicsWorld);

            // The sphere is a moving rigid body
            box.getRigidBody().enableMotion(true);

            // Change the material properties of the rigid body
            Material material = box.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the sphere the list of sphere in the scene
            mBoxes.add(box);
        }

        // Create all the spheres of the scene
        for (int i = 0; i < NB_SPHERES; i++) {

            // Position
            float angle = i * 35.0f;
            Vector3 position = new Vector3(radius * (float) Math.cos(angle),
                    50 + i * (SPHERE_RADIUS + 0.8f),
                    radius * (float) Math.sin(angle));

            // Create a sphere and a corresponding rigid in the dynamics world
            Sphere sphere = new Sphere(SPHERE_RADIUS, position, BOX_MASS, mDynamicsWorld);

            // The sphere is a moving rigid body
            sphere.getRigidBody().enableMotion(true);

            // Change the material properties of the rigid body
            Material material = sphere.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the sphere the list of sphere in the scene
            mSpheres.add(sphere);
        }

        // Create all the cones of the scene
        for (int i = 0; i < NB_CONES; i++) {

            // Position
            float angle = i * 50.0f;
            Vector3 position = new Vector3(radius * (float) Math.cos(angle),
                    35 + i * (CONE_HEIGHT + 0.3f),
                    radius * (float) Math.sin(angle));

            // Create a cone and a corresponding rigid in the dynamics world
            Cone cone = new Cone(CONE_RADIUS, CONE_HEIGHT, position, CONE_MASS, mDynamicsWorld);

            // The cone is a moving rigid body
            cone.getRigidBody().enableMotion(true);

            // Change the material properties of the rigid body
            Material material = cone.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the cone the list of sphere in the scene
            mCones.add(cone);
        }

        // Create all the cylinders of the scene
        for (int i = 0; i < NB_CYLINDERS; i++) {

            // Position
            float angle = i * 35.0f;
            Vector3 position = new Vector3(radius * (float) Math.cos(angle),
                    25 + i * (CYLINDER_HEIGHT + 0.3f),
                    radius * (float) Math.sin(angle));

            // Create a cylinder and a corresponding rigid in the dynamics world
            Cylinder cylinder = new Cylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT, position,
                    CYLINDER_MASS, mDynamicsWorld);

            // The cylinder is a moving rigid body
            cylinder.getRigidBody().enableMotion(true);

            // Change the material properties of the rigid body
            Material material = cylinder.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the cylinder the list of sphere in the scene
            mCylinders.add(cylinder);
        }

        // Create all the capsules of the scene
        for (int i = 0; i < NB_CAPSULES; i++) {

            // Position
            float angle = i * 45.0f;
            Vector3 position = new Vector3(radius * (float) Math.cos(angle),
                    15 + i * (CAPSULE_HEIGHT + 0.3f),
                    radius * (float) Math.sin(angle));

            // Create a cylinder and a corresponding rigid in the dynamics world
            Capsule capsule = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, position,
                    CAPSULE_MASS, mDynamicsWorld);

            // The cylinder is a moving rigid body
            capsule.getRigidBody().enableMotion(true);

            // Change the material properties of the rigid body
            Material material = capsule.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the cylinder the list of sphere in the scene
            mCapsules.add(capsule);
        }

        // Create all the convex meshes of the scene
        for (int i = 0; i < NB_MESHES; i++) {

            // Position
            float angle = i * 30.0f;
            Vector3 position = new Vector3(radius * (float) Math.cos(angle),
                    5 + i * (CAPSULE_HEIGHT + 0.3f),
                    radius * (float) Math.sin(angle));

            // Create a convex mesh and a corresponding rigid in the dynamics world
            ConvexMesh mesh = new ConvexMesh(position, MESH_MASS, mDynamicsWorld);

            // The mesh is a moving rigid body
            mesh.getRigidBody().enableMotion(true);

            // Change the material properties of the rigid body
            Material material = mesh.getRigidBody().getMaterial();
            material.setBounciness(0.2f);

            // Add the mesh the list of sphere in the scene
            mConvexMeshes.add(mesh);
        }

        // Create the floor
        Vector3 floorPosition = new Vector3(0, 0, 0);
        mFloor = new Box(FLOOR_SIZE, floorPosition, FLOOR_MASS, mDynamicsWorld);

        // The floor must be a non-moving rigid body
        mFloor.getRigidBody().enableMotion(false);

        // Change the material properties of the rigid body
        Material material = mFloor.getRigidBody().getMaterial();
        material.setBounciness(0.2f);

        // Start the simulation
        startSimulation();
    }

    // Stop the simulation
    public void stopSimulation() {
        mDynamicsWorld.stop();
        mIsRunning = false;
    }

    // Start the simulation
    public void startSimulation() {
        mDynamicsWorld.start();
        mIsRunning = true;
    }

    // Pause or continue simulation
    public void pauseContinueSimulation() {
        if (mIsRunning) {
            stopSimulation();
        } else {
            startSimulation();
        }
    }

    // Take a step for the simulation
    public void simulate() {

        // If the physics simulation is running
        if (mIsRunning) {

            // Take a simulation step
            mDynamicsWorld.update();

            // Update the position and orientation of the boxes
            for (Box it : mBoxes) {

                // Update the transform used for the rendering
                it.updateTransform();
            }

            // Update the position and orientation of the sphere
            for (Sphere it : mSpheres) {

                // Update the transform used for the rendering
                it.updateTransform();
            }

            // Update the position and orientation of the cones
            for (Cone it : mCones) {

                // Update the transform used for the rendering
                it.updateTransform();
            }

            // Update the position and orientation of the cylinders
            for (Cylinder it : mCylinders) {

                // Update the transform used for the rendering
                it.updateTransform();
            }

            // Update the position and orientation of the capsules
            for (Capsule it : mCapsules) {

                // Update the transform used for the rendering
                it.updateTransform();
            }

            // Update the position and orientation of the convex meshes
            for (ConvexMesh it : mConvexMeshes) {

                // Update the transform used for the rendering
                it.updateTransform();
            }

            // Destroy all the visual contact points
            for (VisualContactPoint it : mContactPoints) {
            }
            mContactPoints.clear();

            // Generate the new visual contact points
            List< ContactManifold> manifolds = mDynamicsWorld.getContactManifolds();
            for (ContactManifold it : manifolds) {
                for (int i = 0; i < it.getNbContactPoints(); i++) {
                    ContactPoint point = it.getContactPoint(i);

                    net.smert.jreactphysics3d.mathematics.Vector3 pos = point.getWorldPointOnBody1();
                    Vector3 position = new Vector3(pos.getX(), pos.getY(), pos.getZ());
                    VisualContactPoint visualPoint = new VisualContactPoint(position);
                    mContactPoints.add(visualPoint);
                }
            }

            mFloor.updateTransform();
        }
    }

    // Render the scene
    public void render() {

        GL11.glEnable(GL11.GL_DEPTH_TEST);
        GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);
        GL11.glEnable(GL11.GL_CULL_FACE);

        // Get the world-space to camera-space matrix
        Camera camera = mViewer.getCamera();
        Matrix4 worldToCameraMatrix = camera.getTransformMatrix().getInverse();

        // Bind the shader
        mPhongShader.bind();

        // Set the variables of the shader
        mPhongShader.setMatrix4x4Uniform("projectionMatrix", camera.getProjectionMatrix());
        mPhongShader.setVector3Uniform("light0PosCameraSpace", worldToCameraMatrix.operatorMultiply(mLight0.getOrigin()));
        mPhongShader.setVector3Uniform("lightAmbientColor", new Vector3(0.3f, 0.3f, 0.3f));
        Color diffColLight0 = mLight0.getDiffuseColor();
        Color specColLight0 = mLight0.getSpecularColor();
        mPhongShader.setVector3Uniform("light0DiffuseColor", new Vector3(diffColLight0.r, diffColLight0.g, diffColLight0.b));
        mPhongShader.setVector3Uniform("light0SpecularColor", new Vector3(specColLight0.r, specColLight0.g, specColLight0.b));
        mPhongShader.setFloatUniform("shininess", 200.0f);

        // Render all the boxes of the scene
        for (Box it : mBoxes) {
            it.render(mPhongShader, worldToCameraMatrix);
        }

        // Render all the sphere of the scene
        for (Sphere it : mSpheres) {
            it.render(mPhongShader, worldToCameraMatrix);
        }

        // Render all the cones of the scene
        for (Cone it : mCones) {
            it.render(mPhongShader, worldToCameraMatrix);
        }

        // Render all the cylinders of the scene
        for (Cylinder it : mCylinders) {
            it.render(mPhongShader, worldToCameraMatrix);
        }

        // Render all the capsules of the scene
        for (Capsule it : mCapsules) {
            it.render(mPhongShader, worldToCameraMatrix);
        }

        // Render all the convex meshes of the scene
        for (ConvexMesh it : mConvexMeshes) {
            it.render(mPhongShader, worldToCameraMatrix);
        }

        // Render all the visual contact points
        for (VisualContactPoint it : mContactPoints) {
            it.render(mPhongShader, worldToCameraMatrix);
        }

        // Render the floor
        mFloor.render(mPhongShader, worldToCameraMatrix);

        // Unbind the shader
        mPhongShader.unbind();
    }

}
