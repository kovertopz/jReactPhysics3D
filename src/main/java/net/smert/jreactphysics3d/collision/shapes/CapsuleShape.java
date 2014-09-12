package net.smert.jreactphysics3d.collision.shapes;

import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a capsule collision shape that is defined around the Y axis. A capsule shape can be seen as the
 * convex hull of two spheres. The capsule shape is defined by its radius (radius of the two spheres of the capsule) and
 * its height (distance between the centers of the two spheres). This collision shape does not have an explicit object
 * margin distance. The margin is implicitly the radius and height of the shape. Therefore, no need to specify an object
 * margin for a capsule shape.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CapsuleShape extends CollisionShape {

    // Radius of the two spheres of the capsule
    private float mRadius;

    // Half height of the capsule (height = distance between the centers of the two spheres)
    private float mHalfHeight;

    // Private copy-constructor
    private CapsuleShape(CapsuleShape shape) {
        super(shape);
        mRadius = shape.mRadius;
        mHalfHeight = shape.mHalfHeight;
    }

    // Constructor
    public CapsuleShape(float radius, float height) {
        // TODO: Should radius really be the margin for a capsule? Seems like a bug.
        super(CollisionShapeType.CAPSULE, radius);

        assert (radius > 0.0f);
        assert (height > 0.0f);

        mRadius = radius;
        mHalfHeight = height * 0.5f;
    }

    // Get the radius of the capsule
    public float getRadius() {
        return mRadius;
    }

    // Return the height of the capsule
    public float getHeight() {
        return mHalfHeight + mHalfHeight;
    }

    @Override
    public CollisionShape clone() {
        return new CapsuleShape(this);
    }

    // Return the local inertia tensor of the capsule
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {

        // The inertia tensor formula for a capsule can be found in : Game Engine Gems, Volume 1
        float height = mHalfHeight + mHalfHeight;
        float radiusSquare = mRadius * mRadius;
        float heightSquare = height * height;
        float radiusSquareDouble = radiusSquare + radiusSquare;
        float factor1 = 2.0f * mRadius / (4.0f * mRadius + 3.0f * height);
        float factor2 = 3.0f * height / (4.0f * mRadius + 3.0f * height);
        float sum1 = 0.4f * radiusSquareDouble;
        float sum2 = 0.75f * height * mRadius + 0.5f * heightSquare;
        float sum3 = 0.25f * radiusSquare + 1.0f / 12.0f * heightSquare;
        float IxxAndzz = factor1 * mass * (sum1 + sum2) + factor2 * mass * sum3;
        float Iyy = factor1 * mass * sum1 + factor2 * mass * 0.25f * radiusSquareDouble;
        tensor.setAllValues(IxxAndzz, 0.0f, 0.0f,
                0.0f, Iyy, 0.0f,
                0.0f, 0.0f, IxxAndzz);
    }

    // Return a local support point in a given direction with the object margin.
    // A capsule is the convex hull of two spheres S1 and S2. The support point in the direction "d"
    // of the convex hull of a set of convex objects is the support point "p" in the set of all
    // support points from all the convex objects with the maximum dot product with the direction "d".
    // Therefore, in this method, we compute the support points of both top and bottom spheres of
    // the capsule and return the point with the maximum dot product with the direction vector. Note
    // that the object margin is implicitly the radius and height of the capsule.
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {

        // If the direction vector is not the zero vector
        if (direction.lengthSquare() >= Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {

            Vector3 unitDirection = direction.getUnit();

            // Support point top sphere
            Vector3 centerTopSphere = new Vector3(0.0f, mHalfHeight, 0.0f);
            Vector3 topSpherePoint = Vector3.operatorAdd(centerTopSphere, unitDirection).operatorMultiplyEqual(mRadius);
            float dotProductTop = topSpherePoint.dot(direction);

            // Support point bottom sphere
            Vector3 centerBottomSphere = new Vector3(0.0f, -mHalfHeight, 0.0f);
            Vector3 bottomSpherePoint = Vector3.operatorAdd(centerBottomSphere, unitDirection).operatorMultiplyEqual(mRadius);
            float dotProductBottom = bottomSpherePoint.dot(direction);

            // Return the point with the maximum dot product
            if (dotProductTop > dotProductBottom) {
                return topSpherePoint;
            } else {
                return bottomSpherePoint;
            }
        }

        // If the direction vector is the zero vector we return a point on the
        // boundary of the capsule
        return new Vector3(0.0f, mRadius, 0.0f);
    }

    // Return a local support point in a given direction without the object margin.
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {

        // If the dot product of the direction and the local Y axis (dotProduct = direction.y)
        // is positive
        if (direction.getY() > 0.0f) {

            // Return the top sphere center point
            return new Vector3(0.0f, mHalfHeight, 0.0f);
        } else {

            // Return the bottom sphere center point
            return new Vector3(0.0f, -mHalfHeight, 0.0f);
        }
    }

    // Return the local bounds of the shape in x, y and z directions
    // This method is used to compute the AABB of the box
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {

        // Maximum bounds
        max.setAllValues(mRadius, mHalfHeight + mRadius, mRadius);

        // Minimum bounds
        min.setAllValues(-mRadius, -max.getY(), -mRadius);
    }

    // Test equality between two capsule shapes
    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        CapsuleShape otherShape = (CapsuleShape) otherCollisionShape;
        return (mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight);
    }

}
