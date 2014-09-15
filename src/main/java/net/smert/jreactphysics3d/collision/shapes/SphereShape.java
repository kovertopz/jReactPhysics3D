package net.smert.jreactphysics3d.collision.shapes;

import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a sphere collision shape that is centered at the origin and defined by its radius. This
 * collision shape does not have an explicit object margin distance. The margin is implicitly the radius of the sphere.
 * Therefore, no need to specify an object margin for a sphere shape.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class SphereShape extends CollisionShape {

    // Radius of the sphere
    private final float radius;

    // Constructor
    public SphereShape(float radius, float margin) {
        super(CollisionShapeType.SPHERE, radius + margin);
        assert (radius > 0.0f);
        this.radius = radius;
    }

    // Copy-constructor
    public SphereShape(SphereShape shape) {
        super(shape);
        radius = shape.radius;
    }

    // Get the radius of the sphere
    public float getRadius() {
        return radius;
    }

    // Update the AABB of a body using its collision shape
    @Override
    public void updateAABB(AABB aabb, Transform transform) {

        // Get the local extents in x,y and z direction
        Vector3 extents = new Vector3(radius, radius, radius);

        // Update the AABB with the new minimum and maximum coordinates
        aabb.setMin(new Vector3(transform.getPosition()).subtract(extents));
        aabb.setMax(new Vector3(transform.getPosition()).add(extents));
    }

    // Test equality between two sphere shapes
    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        SphereShape otherShape = (SphereShape) otherCollisionShape;
        return (radius == otherShape.radius);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction, Vector3 supportPoint) {

        // If the direction vector is not the zero vector
        if (direction.lengthSquare() >= Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {

            // Return the support point of the sphere in the given direction
            return supportPoint.set(direction).normalize().multiply(getMargin());
        }

        // If the direction vector is the zero vector we return a point on the
        // boundary of the sphere
        return supportPoint.set(0.0f, getMargin(), 0.0f);
    }

    // Return a local support point in a given direction without the object margin
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction, Vector3 supportPoint) {

        // Return the center of the sphere (the radius is taken into account in the object margin)
        return supportPoint.zero();
    }

    // Return the local inertia tensor of the sphere
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        float diag = 0.4f * mass * radius * radius;
        tensor.set(diag, 0.0f, 0.0f,
                0.0f, diag, 0.0f,
                0.0f, 0.0f, diag);
    }

    // Return the local bounds of the shape in x, y and z directions.
    // This method is used to compute the AABB of the box
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {

        // Maximum bounds
        max.setX(radius + margin);
        max.setY(max.getX());
        max.setZ(max.getX());

        // Minimum bounds
        min.setX(-radius - margin);
        min.setY(min.getX());
        min.setZ(min.getX());
    }

    @Override
    public CollisionShape clone() {
        return new SphereShape(this);
    }

}
