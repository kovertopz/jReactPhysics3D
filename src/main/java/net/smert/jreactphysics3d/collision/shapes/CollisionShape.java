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
    protected float margin;

    // Current number of similar created shapes
    protected int numSimilarCreatedShapes;

    // Type of the collision shape
    protected final CollisionShapeType type;

    // Constructor
    public CollisionShape(CollisionShapeType type, float margin) {
        assert (margin >= 0.0f);
        this.margin = margin;
        numSimilarCreatedShapes = 0;
        this.type = type;
    }

    // Copy-constructor
    public CollisionShape(CollisionShape shape) {
        margin = shape.margin;
        numSimilarCreatedShapes = shape.numSimilarCreatedShapes;
        type = shape.type;
    }

    // Return the current object margin
    public float getMargin() {
        return margin;
    }

    // Return the number of similar created shapes
    public int getNumSimilarCreatedShapes() {
        return numSimilarCreatedShapes;
    }

    // Decrement the number of similar allocated collision shapes
    public void decrementNumSimilarCreatedShapes() {
        numSimilarCreatedShapes--;
    }

    // Increment the number of similar allocated collision shapes
    public void incrementNumSimilarCreatedShapes() {
        numSimilarCreatedShapes++;
    }

    // Return the type of the collision shape
    public CollisionShapeType getType() {
        return type;
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
    public abstract Vector3 getLocalSupportPointWithMargin(Vector3 direction, Vector3 supportPoint);

    // Return a local support point in a given direction without the object margin
    public abstract Vector3 getLocalSupportPointWithoutMargin(Vector3 direction, Vector3 supportPoint);

    // Return the local inertia tensor of the collision shapes
    public abstract void computeLocalInertiaTensor(Matrix3x3 tensor, float mass);

    // Return the local bounds of the shape in x, y and z directions
    public abstract void getLocalBounds(Vector3 min, Vector3 max);

    @Override
    public abstract CollisionShape clone();

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 67 * hash + Float.floatToIntBits(margin);
        hash = 67 * hash + numSimilarCreatedShapes;
        hash = 67 * hash + Objects.hashCode(type);
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
        if (type == other.type) {
            return false;
        }
        if (Float.floatToIntBits(margin) != Float.floatToIntBits(other.margin)) {
            return false;
        }
        return numSimilarCreatedShapes != other.numSimilarCreatedShapes;
    }

    @Override
    public String toString() {
        return "{type=" + type + " margin=" + margin + "}";
    }

}
