package net.smert.jreactphysics3d.collision.narrowphase;

import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.collision.shapes.SphereShape;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
import net.smert.jreactphysics3d.mathematics.Mathematics;
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
        Vector3 vectorBetweenCenters = new Vector3(transform2.getPosition()).subtract(transform1.getPosition());
        float squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();

        // Compute the sum of the radius
        float sumRadius = sphereShape1.getRadius() + sphereShape2.getRadius();

        // If the sphere collision shapes intersect
        if (squaredDistanceBetweenCenters <= sumRadius * sumRadius) {
            Vector3 centerSphere2InBody1LocalSpace = new Transform(transform1).inverse().multiply(transform2.getPosition(), new Vector3());
            Vector3 centerSphere1InBody2LocalSpace = new Transform(transform2).inverse().multiply(transform1.getPosition(), new Vector3());
            Vector3 intersectionOnBody1 = centerSphere2InBody1LocalSpace.normalize().multiply(sphereShape1.getRadius());
            Vector3 intersectionOnBody2 = centerSphere1InBody2LocalSpace.normalize().multiply(sphereShape2.getRadius());
            float penetrationDepth = sumRadius - Mathematics.Sqrt(squaredDistanceBetweenCenters);

            // Create the contact info object
            contactInfo.normal.set(vectorBetweenCenters.normalize());
            contactInfo.penetrationDepth = penetrationDepth;
            contactInfo.localPoint1.set(intersectionOnBody1);
            contactInfo.localPoint2.set(intersectionOnBody2);

            return true;
        }

        return false;
    }

}
