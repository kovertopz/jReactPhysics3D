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
    private float mRadius;

    // Private copy-constructor
    private SphereShape(SphereShape shape) {
        super(shape);
        mRadius = shape.mRadius;
    }

    // Constructor
    public SphereShape(float radius) {
        super(CollisionShapeType.SPHERE, radius);
        assert (radius > 0.0f);
        mRadius = radius;
    }

    // Get the radius of the sphere
    public float getRadius() {
        return mRadius;
    }

    // Update the AABB of a body using its collision shape
    @Override
    public void updateAABB(AABB aabb, Transform transform) {

        // Get the local extents in x,y and z direction
        Vector3 extents = new Vector3(mRadius, mRadius, mRadius);

        // Update the AABB with the new minimum and maximum coordinates
        aabb.setMin(Vector3.operatorSubtract(transform.getPosition(), extents));
        aabb.setMax(Vector3.operatorAdd(transform.getPosition(), extents));
    }

    @Override
    public CollisionShape clone() {
        return new SphereShape(this);
    }

    // Return the local inertia tensor of the sphere
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        float diag = 0.4f * mass * mRadius * mRadius;
        tensor.setAllValues(diag, 0.0f, 0.0f,
                0.0f, diag, 0.0f,
                0.0f, 0.0f, diag);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {

        // If the direction vector is not the zero vector
        if (direction.lengthSquare() >= Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {

            // Return the support point of the sphere in the given direction
            return direction.getUnit().operatorMultiplyEqual(mMargin);
        }

        // If the direction vector is the zero vector we return a point on the
        // boundary of the sphere
        return new Vector3(0.0f, mMargin, 0.0f);
    }

    // Return a local support point in a given direction without the object margin
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {

        // Return the center of the sphere (the radius is taken into account in the object margin)
        return new Vector3();
    }

    // Return the local bounds of the shape in x, y and z directions.
    // This method is used to compute the AABB of the box
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {

        // Maximum bounds
        max.x = mRadius;
        max.y = max.x;
        max.z = max.x;

        // Minimum bounds
        min.x = -mRadius;
        min.y = min.x;
        min.z = min.x;
    }

    // Test equality between two sphere shapes
    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        SphereShape otherShape = (SphereShape) otherCollisionShape;
        return (mRadius == otherShape.mRadius);
    }

}
