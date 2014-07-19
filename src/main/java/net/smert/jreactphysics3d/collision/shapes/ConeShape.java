package net.smert.jreactphysics3d.collision.shapes;

import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a cone collision shape centered at the origin and alligned with the Y axis. The cone is defined
 * by its height and by the radius of its base. The center of the cone is at the half of the height. The "transform" of
 * the corresponding rigid body gives an orientation and a position to the cone. This collision shape uses an extra
 * margin distance around it for collision detection purpose. The default margin is 4cm (if your units are meters, which
 * is recommended). In case, you want to simulate small objects (smaller than the margin distance), you might want to
 * reduce the margin by specifying your own margin distance using the "margin" parameter in the constructor of the cone
 * shape. Otherwise, it is recommended to use the default margin distance by not using the "margin" parameter in the
 * constructor.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ConeShape extends CollisionShape {

    /// Radius of the base
    private float mRadius;

    /// Half height of the cone
    private float mHalfHeight;

    /// sine of the semi angle at the apex point
    private float mSinTheta;

    // Private copy-constructor
    private ConeShape(ConeShape shape) {
        super(shape);
        mRadius = shape.mRadius;
        mHalfHeight = shape.mHalfHeight;
        mSinTheta = shape.mSinTheta;
    }

    /// Private assignment operator
    private ConeShape operatorEqual(ConeShape shape) {
        return this;
    }

    // Constructor
    public ConeShape(float radius, float height, float margin) {
        super(CollisionShapeType.CONE, margin);

        assert (mRadius > 0.0f);
        assert (mHalfHeight > 0.0f);
        assert (margin > 0.0f);

        mRadius = radius;
        mHalfHeight = height * 0.5f;

        // Compute the sine of the semi-angle at the apex point
        mSinTheta = mRadius / ((float) Math.sqrt(mRadius * mRadius + height * height));
    }

    // Return the radius
    public float getRadius() {
        return mRadius;
    }

    // Return the height
    public float getHeight() {
        return 2.0f * mHalfHeight;
    }

    // Return the number of bytes used by the collision shape
    @Override
    public int getSizeInBytes() {
        return 4;
    }

    // Return the local bounds of the shape in x, y and z directions
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {

        // Maximum bounds
        max.x = mRadius + mMargin;
        max.y = mHalfHeight + mMargin;
        max.z = max.x;

        // Minimum bounds
        min.x = -max.x;
        min.y = -max.y;
        min.z = min.x;
    }

    // Return the local inertia tensor of the collision shape
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        float rSquare = mRadius * mRadius;
        float diagXZ = 0.15f * mass * (rSquare + mHalfHeight);
        tensor.setAllValues(diagXZ, 0.0, 0.0,
                0.0, 0.3f * mass * rSquare,
                0.0, 0.0, 0.0, diagXZ);
    }

    // Test equality between two cone shapes
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        ConeShape otherShape = (ConeShape) otherCollisionShape;
        return (mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {

        // Compute the support point without the margin
        Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);

        // Add the margin to the support point
        Vector3 unitVec = new Vector3(0.0f, -1.0f, 0.0f);
        if (direction.lengthSquare() > Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {
            unitVec = direction.getUnit();
        }
        supportPoint += unitVec * mMargin;

        return supportPoint;
    }

    // Return a local support point in a given direction without the object margin
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {

        Vector3 v = direction;
        float sinThetaTimesLengthV = mSinTheta * v.length();
        Vector3 supportPoint;

        if (v.y > sinThetaTimesLengthV) {
            supportPoint = new Vector3(0.0f, mHalfHeight, 0.0f);
        } else {
            float projectedLength = sqrt(v.x * v.x + v.z * v.z);
            if (projectedLength > Defaults.MACHINE_EPSILON) {
                float d = mRadius / projectedLength;
                supportPoint = new Vector3(v.x * d, -mHalfHeight, v.z * d);
            } else {
                supportPoint = new Vector3(0.0f, -mHalfHeight, 0.0f);
            }
        }

        return supportPoint;
    }

}
