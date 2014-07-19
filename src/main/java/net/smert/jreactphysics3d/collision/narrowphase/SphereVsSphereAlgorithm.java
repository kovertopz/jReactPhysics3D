package net.smert.jreactphysics3d.collision.narrowphase;

import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.collision.shapes.SphereShape;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;
import net.smert.jreactphysics3d.memory.MemoryAllocator;

/**
 * This class is used to compute the narrow-phase collision detection between
 * two sphere collision shapes.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class SphereVsSphereAlgorithm extends NarrowPhaseAlgorithm {

    /// Private copy-constructor
    protected SphereVsSphereAlgorithm(SphereVsSphereAlgorithm algorithm) {
    }

    /// Private assignment operator
    protected SphereVsSphereAlgorithm operatorEqual(SphereVsSphereAlgorithm algorithm) {
        return this;
    }

    // Constructor
    public SphereVsSphereAlgorithm(MemoryAllocator memoryAllocator) {
        super(memoryAllocator);
    }

    @Override
    public boolean testCollision(
            CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            ContactPointInfo contactInfo) {

        // Get the sphere collision shapes
        SphereShape sphereShape1 = (SphereShape) collisionShape1;
        SphereShape sphereShape2 = (SphereShape) collisionShape2;

        // Compute the distance between the centers
        Vector3 vectorBetweenCenters = transform2.getPosition() - transform1.getPosition();
        float squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();

        // Compute the sum of the radius
        float sumRadius = sphereShape1.getRadius() + sphereShape2.getRadius();

        // If the sphere collision shapes intersect
        if (squaredDistanceBetweenCenters <= sumRadius * sumRadius) {
            Vector3 centerSphere2InBody1LocalSpace = transform1.getInverse() * transform2.getPosition();
            Vector3 centerSphere1InBody2LocalSpace = transform2.getInverse() * transform1.getPosition();
            Vector3 intersectionOnBody1 = sphereShape1.getRadius()
                    * centerSphere2InBody1LocalSpace.getUnit();
            Vector3 intersectionOnBody2 = sphereShape2.getRadius()
                    * centerSphere1InBody2LocalSpace.getUnit();
            float penetrationDepth = sumRadius - (float) Math.sqrt(squaredDistanceBetweenCenters);

            // Create the contact info object
            contactInfo = new ContactPointInfo(
                    vectorBetweenCenters.getUnit(), penetrationDepth,
                    intersectionOnBody1, intersectionOnBody2);

            return true;
        }

        return false;
    }

}
