package net.smert.jreactphysics3d.collision.narrowphase.GJK;

import net.smert.jreactphysics3d.collision.narrowphase.EPA.EPAAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.NarrowPhaseAlgorithm;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
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

    // EPA Algorithm
    private EPAAlgorithm mAlgoEPA;

    // This method runs the GJK algorithm on the two enlarged objects (with margin)
    // to compute a simplex polytope that contains the origin. The two objects are
    // assumed to intersect in the original objects (without margin). Therefore such
    // a polytope must exist. Then, we give that polytope to the EPA algorithm to
    // compute the correct penetration depth and contact points of the enlarged objects.
    private boolean computePenetrationDepthForEnlargedObjects(CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            ContactPointInfo contactInfo, Vector3 v) {
        Simplex simplex = new Simplex();
        Vector3 suppA = new Vector3();
        Vector3 suppB = new Vector3();
        Vector3 w = new Vector3();
        float vDotw;
        float distSquare = Defaults.DECIMAL_LARGEST;
        float prevDistSquare;

        // Transform a point from local space of body 2 to local space
        // of body 1 (the GJK algorithm is done in local space of body 1)
        Transform body2ToBody1 = transform1.getInverse().operatorMultiply(transform2);

        // Matrix that transform a direction from local space of body 1 into local space of body 2
        Matrix3x3 rotateToBody2 = Matrix3x3.operatorMultiply(transform2.getOrientation().getMatrix().getTranspose(),
                transform1.getOrientation().getMatrix());

        do {
            // Compute the support points for the enlarged object A and B
            suppA = collisionShape1.getLocalSupportPointWithMargin(new Vector3(v).invert());
            suppB = body2ToBody1.operatorMultiply(collisionShape2.getLocalSupportPointWithMargin(Matrix3x3.operatorMultiply(rotateToBody2, v)));

            // Compute the support point for the Minkowski difference A-B
            w = Vector3.operatorSubtract(suppA, suppB);

            vDotw = v.dot(w);

            // If the enlarge objects do not intersect
            if (vDotw > 0.0f) {

                // No intersection, we return false
                return false;
            }

            // Add the new support point to the simplex
            simplex.addPoint(w, suppA, suppB);

            if (simplex.isAffinelyDependent()) {
                return false;
            }

            if (!simplex.computeClosestPoint(v)) {
                return false;
            }

            // Store and update the square distance
            prevDistSquare = distSquare;
            distSquare = v.lengthSquare();

            if (prevDistSquare - distSquare <= Defaults.MACHINE_EPSILON * prevDistSquare) {
                return false;
            }

        } while (!simplex.isFull() && distSquare > Defaults.MACHINE_EPSILON
                * simplex.getMaxLengthSquareOfAPoint());

        // Give the simplex computed with GJK algorithm to the EPA algorithm
        // which will compute the correct penetration depth and contact points
        // between the two enlarged objects
        return mAlgoEPA.computePenetrationDepthAndContactPoints(simplex, collisionShape1,
                transform1, collisionShape2, transform2,
                v, contactInfo);
    }

    // Constructor
    public GJKAlgorithm() {
        super();
        mAlgoEPA = new EPAAlgorithm();
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
    public boolean testCollision(CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            ContactPointInfo contactInfo) {

        Vector3 suppA;             // Support point of object A
        Vector3 suppB;             // Support point of object B
        Vector3 w;                 // Support point of Minkowski difference A-B
        Vector3 pA = new Vector3();// Closest point of object A
        Vector3 pB = new Vector3();// Closest point of object B
        float vDotw;
        float prevDistSquare;

        // Transform a point from local space of body 2 to local
        // space of body 1 (the GJK algorithm is done in local space of body 1)
        Transform body2Tobody1 = transform1.getInverse().operatorMultiply(transform2);

        // Matrix that transform a direction from local
        // space of body 1 into local space of body 2
        Matrix3x3 rotateToBody2 = Matrix3x3.operatorMultiply(transform2.getOrientation().getMatrix().getTranspose(),
                transform1.getOrientation().getMatrix());

        // Initialize the margin (sum of margins of both objects)
        float margin = collisionShape1.getMargin() + collisionShape2.getMargin();
        float marginSquare = margin * margin;
        assert (margin > 0.0f);

        // Create a simplex set
        Simplex simplex = new Simplex();

        // Get the previous point V (last cached separating axis)
        Vector3 v = mCurrentOverlappingPair.previousSeparatingAxis;

        // Initialize the upper bound for the square distance
        float distSquare = Defaults.DECIMAL_LARGEST;

        do {

            // Compute the support points for original objects (without margins) A and B
            suppA = collisionShape1.getLocalSupportPointWithoutMargin(new Vector3(v).invert());
            suppB = body2Tobody1.operatorMultiply(collisionShape2.getLocalSupportPointWithoutMargin(Matrix3x3.operatorMultiply(rotateToBody2, v)));

            // Compute the support point for the Minkowski difference A-B
            w = Vector3.operatorSubtract(suppA, suppB);

            vDotw = v.dot(w);

            // If the enlarge objects (with margins) do not intersect
            if (vDotw > 0.0f && vDotw * vDotw > distSquare * marginSquare) {

                // Cache the current separating axis for frame coherence
                mCurrentOverlappingPair.previousSeparatingAxis = v;

                // No intersection, we return false
                return false;
            }

            // If the objects intersect only in the margins
            if (simplex.isPointInSimplex(w) || distSquare - vDotw <= distSquare * REL_ERROR_SQUARE) {

                // Compute the closet points of both objects (without the margins)
                simplex.computeClosestPointsOfAandB(pA, pB);

                // Project those two points on the margins to have the closest points of both
                // object with the margins
                float dist = (float) Math.sqrt(distSquare);
                assert (dist > 0.0f);
                pA = Vector3.operatorSubtract(pA, Vector3.operatorMultiply(collisionShape1.getMargin() / dist, v));
                pB = body2Tobody1.getInverse().operatorMultiply(Vector3.operatorAdd(pB, Vector3.operatorMultiply(collisionShape2.getMargin() / dist, v)));

                // Compute the contact info
                Vector3 normal = Matrix3x3.operatorMultiply(transform1.getOrientation().getMatrix(), new Vector3(v.getUnit()).invert());
                float penetrationDepth = margin - dist;

                // Reject the contact if the penetration depth is negative (due too numerical errors)
                if (penetrationDepth <= 0.0f) {
                    return false;
                }

                // Create the contact info object
                contactInfo.normal = normal;
                contactInfo.penetrationDepth = penetrationDepth;
                contactInfo.localPoint1 = pA;
                contactInfo.localPoint2 = pB;

                // There is an intersection, therefore we return true
                return true;
            }

            // Add the new support point to the simplex
            simplex.addPoint(w, suppA, suppB);

            // If the simplex is affinely dependent
            if (simplex.isAffinelyDependent()) {

                // Compute the closet points of both objects (without the margins)
                simplex.computeClosestPointsOfAandB(pA, pB);

                // Project those two points on the margins to have the closest points of both
                // object with the margins
                float dist = (float) Math.sqrt(distSquare);
                assert (dist > 0.0f);
                pA = Vector3.operatorSubtract(pA, Vector3.operatorMultiply(collisionShape1.getMargin() / dist, v));
                pB = body2Tobody1.getInverse().operatorMultiply(Vector3.operatorAdd(pB, Vector3.operatorMultiply(collisionShape2.getMargin() / dist, v)));

                // Compute the contact info
                Vector3 normal = Matrix3x3.operatorMultiply(transform1.getOrientation().getMatrix(), new Vector3(v.getUnit()).invert());
                float penetrationDepth = margin - dist;

                // Reject the contact if the penetration depth is negative (due too numerical errors)
                if (penetrationDepth <= 0.0f) {
                    return false;
                }

                // Create the contact info object
                contactInfo.normal = normal;
                contactInfo.penetrationDepth = penetrationDepth;
                contactInfo.localPoint1 = pA;
                contactInfo.localPoint2 = pB;

                // There is an intersection, therefore we return true
                return true;
            }

            // Compute the point of the simplex closest to the origin
            // If the computation of the closest point fail
            if (!simplex.computeClosestPoint(v)) {

                // Compute the closet points of both objects (without the margins)
                simplex.computeClosestPointsOfAandB(pA, pB);

                // Project those two points on the margins to have the closest points of both
                // object with the margins
                float dist = (float) Math.sqrt(distSquare);
                assert (dist > 0.0f);
                pA = Vector3.operatorSubtract(pA, Vector3.operatorMultiply(collisionShape1.getMargin() / dist, v));
                pB = body2Tobody1.getInverse().operatorMultiply(Vector3.operatorAdd(pB, Vector3.operatorMultiply(collisionShape2.getMargin() / dist, v)));

                // Compute the contact info
                Vector3 normal = Matrix3x3.operatorMultiply(transform1.getOrientation().getMatrix(), new Vector3(v.getUnit()).invert());
                float penetrationDepth = margin - dist;

                // Reject the contact if the penetration depth is negative (due too numerical errors)
                if (penetrationDepth <= 0.0f) {
                    return false;
                }

                // Create the contact info object
                contactInfo.normal = normal;
                contactInfo.penetrationDepth = penetrationDepth;
                contactInfo.localPoint1 = pA;
                contactInfo.localPoint2 = pB;

                // There is an intersection, therefore we return true
                return true;
            }

            // Store and update the squared distance of the closest point
            prevDistSquare = distSquare;
            distSquare = v.lengthSquare();

            // If the distance to the closest point doesn't improve a lot
            if (prevDistSquare - distSquare <= Defaults.MACHINE_EPSILON * prevDistSquare) {
                simplex.backupClosestPointInSimplex(v);

                // Get the new squared distance
                distSquare = v.lengthSquare();

                // Compute the closet points of both objects (without the margins)
                simplex.computeClosestPointsOfAandB(pA, pB);

                // Project those two points on the margins to have the closest points of both
                // object with the margins
                float dist = (float) Math.sqrt(distSquare);
                assert (dist > 0.0f);
                pA = Vector3.operatorSubtract(pA, Vector3.operatorMultiply(collisionShape1.getMargin() / dist, v));
                pB = body2Tobody1.getInverse().operatorMultiply(Vector3.operatorAdd(pB, Vector3.operatorMultiply(collisionShape2.getMargin() / dist, v)));

                // Compute the contact info
                Vector3 normal = Matrix3x3.operatorMultiply(transform1.getOrientation().getMatrix(), new Vector3(v.getUnit()).invert());
                float penetrationDepth = margin - dist;

                // Reject the contact if the penetration depth is negative (due too numerical errors)
                if (penetrationDepth <= 0.0f) {
                    return false;
                }

                // Create the contact info object
                contactInfo.normal = normal;
                contactInfo.penetrationDepth = penetrationDepth;
                contactInfo.localPoint1 = pA;
                contactInfo.localPoint2 = pB;

                // There is an intersection, therefore we return true
                return true;
            }
        } while (!simplex.isFull() && distSquare > Defaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());

        // The objects (without margins) intersect. Therefore, we run the GJK algorithm
        // again but on the enlarged objects to compute a simplex polytope that contains
        // the origin. Then, we give that simplex polytope to the EPA algorithm to compute
        // the correct penetration depth and contact points between the enlarged objects.
        return computePenetrationDepthForEnlargedObjects(collisionShape1, transform1,
                collisionShape2, transform2, contactInfo, v);
    }

}
