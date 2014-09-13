package net.smert.jreactphysics3d.collision.shapes;

import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a cylinder collision shape around the Y axis and centered at the origin. The cylinder is
 * defined by its height and the radius of its base. The "transform" of the corresponding rigid body gives an
 * orientation and a position to the cylinder. This collision shape uses an extra margin distance around it for
 * collision detection purpose. The default margin is 4cm (if your units are meters, which is recommended). In case, you
 * want to simulate small objects (smaller than the margin distance), you might want to reduce the margin by specifying
 * your own margin distance using the "margin" parameter in the constructor of the cylinder shape. Otherwise, it is
 * recommended to use the default margin distance by not using the "margin" parameter in the constructor.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CylinderShape extends CollisionShape {

    // Radius of the base
    private float mRadius;

    // Half height of the cylinder
    private float mHalfHeight;

    // Private copy-constructor
    private CylinderShape(CylinderShape shape) {
        super(shape);
        mRadius = shape.mRadius;
        mHalfHeight = shape.mHalfHeight;
    }

    // Constructor
    public CylinderShape(float radius, float height, float margin) {
        super(CollisionShapeType.CYLINDER, margin);

        assert (radius > 0.0f);
        assert (height > 0.0f);

        mRadius = radius;
        mHalfHeight = height / 2.0f;
    }

    // Return the radius
    public float getRadius() {
        return mRadius;
    }

    // Return the height
    public float getHeight() {
        return mHalfHeight + mHalfHeight;
    }

    @Override
    public CollisionShape clone() {
        return new CylinderShape(this);
    }

    // Return the local inertia tensor of the cylinder
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        float height = 2.0f * mHalfHeight;
        float diag = (1.0f / 12.0f) * mass * (3 * mRadius * mRadius + height * height);
        tensor.set(diag, 0.0f, 0.0f,
                0.0f, 0.5f * mass * mRadius * mRadius, 0.0f,
                0.0f, 0.0f, diag);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {

        // Compute the support point without the margin
        Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);

        // Add the margin to the support point
        Vector3 unitVec = new Vector3(0.0f, 1.0f, 0.0f);
        if (direction.lengthSquare() > Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {
            unitVec = new Vector3(direction).normalize();
        }
        supportPoint.add(unitVec.multiply(mMargin));

        return supportPoint;
    }

    // Return a local support point in a given direction without the object margin
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {

        Vector3 supportPoint = new Vector3();
        float uDotv = direction.getY();
        Vector3 w = new Vector3(direction.getX(), 0.0f, direction.getZ());
        float lengthW = (float) Math.sqrt(direction.getX() * direction.getX() + direction.getZ() * direction.getZ());

        if (lengthW > Defaults.MACHINE_EPSILON) {
            if (uDotv < 0.0f) {
                supportPoint.setY(-mHalfHeight);
            } else {
                supportPoint.setY(mHalfHeight);
            }
            supportPoint.add(w.multiply(mRadius / lengthW));
        } else {
            if (uDotv < 0.0f) {
                supportPoint.setY(-mHalfHeight);
            } else {
                supportPoint.setY(mHalfHeight);
            }
        }

        return supportPoint;
    }

    // Return the local bounds of the shape in x, y and z directions
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {

        // Maximum bounds
        max.setX(mRadius + mMargin);
        max.setY(mHalfHeight + mMargin);
        max.setZ(max.getX());

        // Minimum bounds
        min.setX(-max.getX());
        min.setY(-max.getY());
        min.setZ(min.getX());
    }

    // Test equality between two cylinder shapes
    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        CylinderShape otherShape = (CylinderShape) otherCollisionShape;
        return (mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight);
    }

}
