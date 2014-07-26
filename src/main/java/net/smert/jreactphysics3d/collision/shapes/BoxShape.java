package net.smert.jreactphysics3d.collision.shapes;

import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a 3D box shape. Those axis are unit length. The three extents are half-widths of the box along
 * the three axis x, y, z local axis. The "transform" of the corresponding rigid body will give an orientation and a
 * position to the box. This collision shape uses an extra margin distance around it for collision detection purpose.
 * The default margin is 4cm (if your units are meters, which is recommended). In case, you want to simulate small
 * objects (smaller than the margin distance), you might want to reduce the margin by specifying your own margin
 * distance using the "margin" parameter in the constructor of the box shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BoxShape extends CollisionShape {

    // Extent sizes of the box in the x, y and z direction
    private final Vector3 mExtent;

    // Private copy-constructor
    private BoxShape(BoxShape shape) {
        super(shape);
        mExtent = new Vector3();
        mExtent.setAllValues(shape.mExtent.x, shape.mExtent.y, shape.mExtent.z);
    }

    // Constructor
    public BoxShape(Vector3 extent, float margin) {
        super(CollisionShapeType.BOX, margin);

        assert (extent.x > 0.0f && extent.x > margin);
        assert (extent.y > 0.0f && extent.y > margin);
        assert (extent.z > 0.0f && extent.z > margin);

        mExtent = Vector3.operatorSubtract(extent, new Vector3(margin, margin, margin));
    }

    // Return the extents of the box
    public Vector3 getExtent() {
        return Vector3.operatorAdd(mExtent, new Vector3(mMargin, mMargin, mMargin));
    }

    @Override
    public CollisionShape clone() {
        return new BoxShape(this);
    }

    // Return the local inertia tensor of the collision shape
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        float factor = (1.0f / 3.0f) * mass;
        Vector3 realExtent = Vector3.operatorAdd(mExtent, new Vector3(mMargin, mMargin, mMargin));
        float xSquare = realExtent.x * realExtent.x;
        float ySquare = realExtent.y * realExtent.y;
        float zSquare = realExtent.z * realExtent.z;
        tensor.setAllValues(factor * (ySquare + zSquare), 0.0f, 0.0f,
                0.0f, factor * (xSquare + zSquare), 0.0f,
                0.0f, 0.0f, factor * (xSquare + ySquare));
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {

        assert (mMargin > 0.0f);

        return new Vector3(direction.x < 0.0f ? -mExtent.x - mMargin : mExtent.x + mMargin,
                direction.y < 0.0f ? -mExtent.y - mMargin : mExtent.y + mMargin,
                direction.z < 0.0f ? -mExtent.z - mMargin : mExtent.z + mMargin);
    }

    // Return a local support point in a given direction without the objec margin
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {

        return new Vector3(direction.x < 0.0f ? -mExtent.x : mExtent.x,
                direction.y < 0.0f ? -mExtent.y : mExtent.y,
                direction.z < 0.0f ? -mExtent.z : mExtent.z);
    }

    // Return the local bounds of the shape in x, y and z directions
    // This method is used to compute the AABB of the box
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {

        // Maximum bounds
        max.setAllValues(mExtent.x, mExtent.y, mExtent.z);
        max.operatorAddEqual(new Vector3(mMargin, mMargin, mMargin));

        // Minimum bounds
        min.setAllValues(-max.x, -max.y, -max.z);
    }

    // Test equality between two box shapes
    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        BoxShape otherShape = (BoxShape) otherCollisionShape;
        return (mExtent.equals(otherShape.mExtent));
    }

}
