package net.smert.jreactphysics3d.collision.shapes;

import java.util.Objects;
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

    // Margin used for the GJK collision detection algorithm
    protected float mMargin;

    // Current number of similar created shapes
    protected int mNbSimilarCreatedShapes;

    // Type of the collision shape
    protected final CollisionShapeType mType;

    // Constructor
    public CollisionShape(CollisionShapeType type, float margin) {
        assert (margin >= 0.0f);
        mMargin = margin;
        mNbSimilarCreatedShapes = 0;
        mType = type;
    }

    // Copy-constructor
    public CollisionShape(CollisionShape shape) {
        mMargin = shape.mMargin;
        mNbSimilarCreatedShapes = shape.mNbSimilarCreatedShapes;
        mType = shape.mType;
    }

    // Return the current object margin
    public float getMargin() {
        return mMargin;
    }

    // Return the number of similar created shapes
    public int getNbSimilarCreatedShapes() {
        return mNbSimilarCreatedShapes;
    }

    // Decrement the number of similar allocated collision shapes
    public void decrementNbSimilarCreatedShapes() {
        mNbSimilarCreatedShapes--;
    }

    // Increment the number of similar allocated collision shapes
    public void incrementNbSimilarCreatedShapes() {
        mNbSimilarCreatedShapes++;
    }

    // Return the type of the collision shape
    public CollisionShapeType getType() {
        return mType;
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
        Matrix3x3 worldAxis = transform.getOrientation().getMatrix(new Matrix3x3());
        worldAxis.abs();
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

    // Test equality between two collision shapes of the same type (same derived classes).
    public abstract boolean isEqualTo(CollisionShape otherCollisionShape);

    // Return a local support point in a given direction with the object margin
    public abstract Vector3 getLocalSupportPointWithMargin(Vector3 direction);

    // Return a local support point in a given direction without the object margin
    public abstract Vector3 getLocalSupportPointWithoutMargin(Vector3 direction);

    // Return the local inertia tensor of the collision shapes
    public abstract void computeLocalInertiaTensor(Matrix3x3 tensor, float mass);

    // Return the local bounds of the shape in x, y and z directions
    public abstract void getLocalBounds(Vector3 min, Vector3 max);

    @Override
    public abstract CollisionShape clone();

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 67 * hash + Float.floatToIntBits(this.mMargin);
        hash = 67 * hash + this.mNbSimilarCreatedShapes;
        hash = 67 * hash + Objects.hashCode(this.mType);
        return hash;
    }

    @Override
    public boolean equals(Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final CollisionShape other = (CollisionShape) obj;
        if (this.mType == other.mType) {
            return false;
        }
        if (Float.floatToIntBits(this.mMargin) != Float.floatToIntBits(other.mMargin)) {
            return false;
        }
        return this.mNbSimilarCreatedShapes != other.mNbSimilarCreatedShapes;
    }

    @Override
    public String toString() {
        return "";
    }

}
