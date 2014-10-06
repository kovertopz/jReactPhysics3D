/*
 * ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/
 * Copyright (c) 2010-2013 Daniel Chappuis
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim
 *    that you wrote the original software. If you use this software in a
 *    product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * This file has been modified during the port to Java and differ from the source versions.
 */
package net.smert.jreactphysics3d.collision.narrowphase.GJK;

import net.smert.jreactphysics3d.collision.narrowphase.EPA.EPAAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.NarrowPhaseAlgorithm;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
import net.smert.jreactphysics3d.mathematics.Mathematics;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class implements a narrow-phase collision detection algorithm. This algorithm uses the ISA-GJK algorithm and the
 * EPA algorithm. This implementation is based on the implementation discussed in the book "Collision Detection in
 * Interactive 3D Environments" by Gino van den Bergen. This method implements the Hybrid Technique for calculating the
 * penetration depth. The two objects are enlarged with a small margin. If the object intersects in their margins, the
 * penetration depth is quickly computed using the GJK algorithm on the original objects (without margin). If the
 * original objects (without margin) intersect, we run again the GJK algorithm on the enlarged objects (with margin) to
 * compute simplex polytope that contains the origin and give it to the EPA (Expanding Polytope Algorithm) to compute
 * the correct penetration depth between the enlarged objects.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class GJKAlgorithm extends NarrowPhaseAlgorithm {

    private static final float REL_ERROR = 1.0e-3f;
    public static final float REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;

    private float distanceSquare;

    // Sum of margins of both objects
    private float margin;

    // EPA Algorithm
    private final EPAAlgorithm epaAlgorithm;

    // Matrix that transform a direction from local space of body 1 into local space of body 2
    private final Matrix3x3 rotateToBody2;

    // Transform a point from local space of body 2 to local space of body 1
    private final Transform body2ToBody1;

    // Support point direction
    private final Vector3 direction;

    // Support point of Minkowski difference supportA-supportB
    private final Vector3 minkDiff;

    // Closest point of object A
    private final Vector3 pointA;

    // Closest point of object B
    private final Vector3 pointB;

    // Support point direction (relative to the shape)
    private final Vector3 relativeDirection;

    // Support point of object A
    private final Vector3 supportA;

    // Support point of object B
    private final Vector3 supportB;

    // Constructor
    public GJKAlgorithm() {
        super();
        epaAlgorithm = new EPAAlgorithm();
        rotateToBody2 = new Matrix3x3();
        body2ToBody1 = new Transform();
        direction = new Vector3();
        minkDiff = new Vector3();
        pointA = new Vector3();
        pointB = new Vector3();
        relativeDirection = new Vector3();
        supportA = new Vector3();
        supportB = new Vector3();
    }

    private boolean addContactInfo(
            CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            ContactPointInfo contactInfo, Simplex simplex) {

        // Compute the closet points of both objects (without the margins)
        simplex.computeClosestPointsOfAandB(pointA, pointB);

        // Project those two points on the margins to have the closest points of both object with the margins
        float distance = Mathematics.Sqrt(distanceSquare);
        assert (distance > 0.0f);
        pointA.subtract(new Vector3(direction).multiply(collisionShape1.getMargin() / distance));
        pointB.add(new Vector3(direction).multiply(collisionShape2.getMargin() / distance));
        pointB.set(body2ToBody1.inverse().multiply(pointB, new Vector3()));

        // Compute the contact info
        Matrix3x3 tempRotation = transform1.getOrientation().getMatrix(new Matrix3x3());
        Vector3 normal = tempRotation.multiply(new Vector3(direction).normalize().invert(), new Vector3());
        float penetrationDepth = margin - distance;

        // Reject the contact if the penetration depth is negative (due too numerical errors)
        if (penetrationDepth <= 0.0f) {
            return false;
        }

        // Create the contact info object
        contactInfo.setCollisionData(normal, penetrationDepth, pointA, pointB);

        // There is an intersection, therefore we return true
        return true;
    }

    // This method runs the GJK algorithm on the two enlarged objects (with margin)
    // to compute a simplex polytope that contains the origin. The two objects are
    // assumed to intersect in the original objects (without margin). Therefore such
    // a polytope must exist. Then, we give that polytope to the EPA algorithm to
    // compute the correct penetration depth and contact points of the enlarged objects.
    private boolean computePenetrationDepthForEnlargedObjects(
            CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            ContactPointInfo contactInfo) {

        Simplex simplex = new Simplex();

        distanceSquare = Defaults.DECIMAL_LARGEST;
        float directionDotMinkDiff, prevDistanceSquare;

        do {

            // Compute the support points for enlarged objects (with margins) A and B
            relativeDirection.set(direction).invert();
            supportA.set(collisionShape1.getLocalSupportPointWithMargin(relativeDirection, new Vector3()));

            rotateToBody2.multiply(direction, relativeDirection);
            supportB.set(collisionShape2.getLocalSupportPointWithMargin(relativeDirection, new Vector3()));
            supportB.set(body2ToBody1.multiply(supportB, new Vector3()));

            // Compute the support point for the Minkowski difference A-B
            minkDiff.set(supportA).subtract(supportB);

            directionDotMinkDiff = direction.dot(minkDiff);

            // If the enlarge objects do not intersect
            if (directionDotMinkDiff > 0.0f) {

                // No intersection, we return false
                return false;
            }

            // Add the new support point to the simplex
            simplex.addPoint(minkDiff, supportA, supportB);

            if (simplex.isAffinelyDependent()) {
                return false;
            }

            if (!simplex.computeClosestPoint(direction)) {
                return false;
            }

            // Store and update the square distance
            prevDistanceSquare = distanceSquare;
            distanceSquare = direction.lengthSquare();

            if (prevDistanceSquare - distanceSquare <= Defaults.MACHINE_EPSILON * prevDistanceSquare) {
                return false;
            }

        } while (!simplex.isFull() && distanceSquare > Defaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());

        // Give the simplex computed with GJK algorithm to the EPA algorithm
        // which will compute the correct penetration depth and contact points
        // between the two enlarged objects
        return epaAlgorithm.computePenetrationDepthAndContactPoints(
                collisionShape1, transform1, collisionShape2, transform2, contactInfo, simplex);
    }

    // Return true and compute a contact info if the two bounding volumes collide.
    // This method implements the Hybrid Technique for computing the penetration depth by
    // running the GJK algorithm on original objects (without margin).
    // If the objects don't intersect, this method returns false. If they intersect
    // only in the margins, the method compute the penetration depth and contact points
    // (of enlarged objects). If the original objects (without margin) intersect, we
    // call the computePenetrationDepthForEnlargedObjects() method that run the GJK
    // algorithm on the enlarged object to obtain a simplex polytope that contains the
    // origin, they we give that simplex polytope to the EPA algorithm which will compute
    // the correct penetration depth and contact points between the enlarged objects.
    @Override
    public boolean testCollision(
            CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            ContactPointInfo contactInfo) {

        // Create a simplex set
        Simplex simplex = new Simplex();

        // Transform a point from local space of body 2 to local
        // space of body 1 (the GJK algorithm is done in local space of body 1)
        body2ToBody1.set(transform1).inverse().multiply(transform2);

        // Matrix that transform a direction from local
        // space of body 1 into local space of body 2
        Matrix3x3 tempRotation1 = transform1.getOrientation().getMatrix(new Matrix3x3());
        Matrix3x3 tempRotation2 = transform2.getOrientation().getMatrix(new Matrix3x3());
        rotateToBody2.set(tempRotation2.transpose()).multiply(tempRotation1);

        // Get the previous point (last cached separating axis)
        direction.set(currentOverlappingPair.getPreviousSeparatingAxis());

        // Initialize the upper bound for the square distance
        distanceSquare = Defaults.DECIMAL_LARGEST;

        // Initialize the margin (sum of margins of both objects)
        margin = collisionShape1.getMargin() + collisionShape2.getMargin();
        assert (margin > 0.0f);

        float directionDotMinkDiff, prevDistanceSquare;
        float marginSquare = margin * margin;

        do {

            // Compute the support points for original objects (without margins) A and B
            relativeDirection.set(direction).invert();
            supportA.set(collisionShape1.getLocalSupportPointWithoutMargin(relativeDirection, new Vector3()));

            rotateToBody2.multiply(direction, relativeDirection);
            supportB.set(collisionShape2.getLocalSupportPointWithoutMargin(relativeDirection, new Vector3()));
            supportB.set(body2ToBody1.multiply(supportB, new Vector3()));

            // Compute the support point for the Minkowski difference A-B
            minkDiff.set(supportA).subtract(supportB);

            directionDotMinkDiff = direction.dot(minkDiff);

            // If the enlarge objects (with margins) do not intersect
            if ((directionDotMinkDiff > 0.0f)
                    && (directionDotMinkDiff * directionDotMinkDiff > distanceSquare * marginSquare)) {

                // Cache the current separating axis for frame coherence
                currentOverlappingPair.getPreviousSeparatingAxis().set(direction);

                // No intersection, we return false
                return false;
            }

            // If the objects intersect only in the margins
            if ((simplex.isPointInSimplex(minkDiff))
                    || (distanceSquare - directionDotMinkDiff <= distanceSquare * REL_ERROR_SQUARE)) {
                return addContactInfo(collisionShape1, transform1, collisionShape2, transform2, contactInfo, simplex);
            }

            // Add the new support point to the simplex
            simplex.addPoint(minkDiff, supportA, supportB);

            // If the simplex is affinely dependent
            if (simplex.isAffinelyDependent()) {
                return addContactInfo(collisionShape1, transform1, collisionShape2, transform2, contactInfo, simplex);
            }

            // Compute the point of the simplex closest to the origin
            // If the computation of the closest point fail
            if (!simplex.computeClosestPoint(direction)) {
                return addContactInfo(collisionShape1, transform1, collisionShape2, transform2, contactInfo, simplex);
            }

            // Store and update the squared distance of the closest point
            prevDistanceSquare = distanceSquare;
            distanceSquare = direction.lengthSquare();

            // If the distance to the closest point doesn't improve a lot
            if (prevDistanceSquare - distanceSquare <= Defaults.MACHINE_EPSILON * prevDistanceSquare) {
                simplex.backupClosestPointInSimplex(direction);

                // Get the new squared distance
                distanceSquare = direction.lengthSquare();

                return addContactInfo(collisionShape1, transform1, collisionShape2, transform2, contactInfo, simplex);
            }

        } while (!simplex.isFull() && distanceSquare > Defaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());

        // The objects (without margins) intersect. Therefore, we run the GJK algorithm
        // again but on the enlarged objects to compute a simplex polytope that contains
        // the origin. Then, we give that simplex polytope to the EPA algorithm to compute
        // the correct penetration depth and contact points between the enlarged objects.
        return computePenetrationDepthForEnlargedObjects(collisionShape1, transform1, collisionShape2, transform2, contactInfo);
    }

}
