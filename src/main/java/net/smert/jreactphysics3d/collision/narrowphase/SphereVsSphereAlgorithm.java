package net.smert.jreactphysics3d.collision.narrowphase;

import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.collision.shapes.SphereShape;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class is used to compute the narrow-phase collision detection between two sphere collision shapes.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class SphereVsSphereAlgorithm extends NarrowPhaseAlgorithm {

    // Constructor
    public SphereVsSphereAlgorithm() {
        super();
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
        Vector3 vectorBetweenCenters = Vector3.operatorSubtract(transform2.getPosition(), transform1.getPosition());
        float squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();

        // Compute the sum of the radius
        float sumRadius = sphereShape1.getRadius() + sphereShape2.getRadius();

        // If the sphere collision shapes intersect
        if (squaredDistanceBetweenCenters <= sumRadius * sumRadius) {
            Vector3 centerSphere2InBody1LocalSpace = transform1.getInverse().operatorMultiply(transform2.getPosition());
            Vector3 centerSphere1InBody2LocalSpace = transform2.getInverse().operatorMultiply(transform1.getPosition());
            Vector3 intersectionOnBody1 = centerSphere2InBody1LocalSpace.normalize().multiply(sphereShape1.getRadius());
            Vector3 intersectionOnBody2 = centerSphere1InBody2LocalSpace.normalize().multiply(sphereShape2.getRadius());
            float penetrationDepth = sumRadius - (float) Math.sqrt(squaredDistanceBetweenCenters);

            // Create the contact info object
            contactInfo.normal = vectorBetweenCenters.normalize();
            contactInfo.penetrationDepth = penetrationDepth;
            contactInfo.localPoint1 = intersectionOnBody1;
            contactInfo.localPoint2 = intersectionOnBody2;

            return true;
        }

        return false;
    }

}
