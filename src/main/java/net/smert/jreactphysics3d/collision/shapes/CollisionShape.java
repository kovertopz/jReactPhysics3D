package net.smert.jreactphysics3d.collision.shapes;

import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This abstract class represents the collision shape associated with a body that is used during the narrow-phase
 * collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class CollisionShape {

    // Type of the collision shape
    protected final CollisionShapeType mType;

    // Current number of similar created shapes
    protected int mNbSimilarCreatedShapes;

    // Margin used for the GJK collision detection algorithm
    protected float mMargin;

    // Private copy-constructor
    protected CollisionShape(CollisionShape shape) {
        mType = shape.mType;
        mNbSimilarCreatedShapes = shape.mNbSimilarCreatedShapes;
        mMargin = shape.mMargin;
    }

    // Constructor
    public CollisionShape(CollisionShapeType type, float margin) {
        mType = type;
        mNbSimilarCreatedShapes = 0;
        mMargin = margin;

        assert (margin > 0.0f);
    }

    // Return the type of the collision shape
    public CollisionShapeType getType() {
        return mType;
    }

    // Return the number of similar created shapes
    public int getNbSimilarCreatedShapes() {
        return mNbSimilarCreatedShapes;
    }

    // Increment the number of similar allocated collision shapes
    public void incrementNbSimilarCreatedShapes() {
        mNbSimilarCreatedShapes++;
    }

    // Decrement the number of similar allocated collision shapes
    public void decrementNbSimilarCreatedShapes() {
        mNbSimilarCreatedShapes--;
    }

    // Return the current object margin
    public float getMargin() {
        return mMargin;
    }

    // Equality operator between two collision shapes.
    // This methods returns true only if the two collision shapes are of the same type and
    // of the same dimensions.
    public boolean operatorEquals(CollisionShape otherCollisionShape) {

        // If the two collisions shapes are not of the same type (same derived classes)
        // we return false
        if (mType != otherCollisionShape.mType) {
            return false;
        }

        assert (this.getClass().equals(otherCollisionShape.getClass()));

        if (mMargin != otherCollisionShape.mMargin) {
            return false;
        }

        // Check if the two shapes are equal
        return otherCollisionShape.isEqualTo(this);
    }

    // Update the AABB of a body using its collision shape
    public void updateAABB(AABB aabb, Transform transform) {

        // Get the local bounds in x,y and z direction
        Vector3 minBounds = new Vector3();
        Vector3 maxBounds = new Vector3();
        getLocalBounds(minBounds, maxBounds);

        // Rotate the local bounds according to the orientation of the body
        Matrix3x3 worldAxis = new Matrix3x3();
        transform.getOrientation().getMatrix(worldAxis);
        worldAxis.getAbsoluteMatrix();
        Vector3 worldMinBounds = new Vector3(worldAxis.getColumn(0).dot(minBounds),
                worldAxis.getColumn(1).dot(minBounds),
                worldAxis.getColumn(2).dot(minBounds));
        Vector3 worldMaxBounds = new Vector3(worldAxis.getColumn(0).dot(maxBounds),
                worldAxis.getColumn(1).dot(maxBounds),
                worldAxis.getColumn(2).dot(maxBounds));

        // Compute the minimum and maximum coordinates of the rotated extents
        Vector3 minCoordinates = new Vector3(transform.getPosition()).add(worldMinBounds);
        Vector3 maxCoordinates = new Vector3(transform.getPosition()).add(worldMaxBounds);

        // Update the AABB with the new minimum and maximum coordinates
        aabb.setMin(minCoordinates);
        aabb.setMax(maxCoordinates);
    }

    @Override
    public abstract CollisionShape clone();

    // Return the local inertia tensor of the collision shapes
    public abstract void computeLocalInertiaTensor(Matrix3x3 tensor, float mass);

    // Return a local support point in a given direction with the object margin
    public abstract Vector3 getLocalSupportPointWithMargin(Vector3 direction);

    // Return a local support point in a given direction without the object margin
    public abstract Vector3 getLocalSupportPointWithoutMargin(Vector3 direction);

    // Return the local bounds of the shape in x, y and z directions
    public abstract void getLocalBounds(Vector3 min, Vector3 max);

    // Test equality between two collision shapes of the same type (same derived classes).
    public abstract boolean isEqualTo(CollisionShape otherCollisionShape);

}
