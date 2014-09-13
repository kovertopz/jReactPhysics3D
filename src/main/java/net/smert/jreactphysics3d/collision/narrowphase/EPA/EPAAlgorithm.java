package net.smert.jreactphysics3d.collision.narrowphase.EPA;

import java.util.PriorityQueue;
import java.util.Queue;
import net.smert.jreactphysics3d.collision.narrowphase.GJK.GJKAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.GJK.Simplex;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class is the implementation of the Expanding Polytope Algorithm (EPA). The EPA algorithm computes the
 * penetration depth and contact points between two enlarged objects (with margin) where the original objects (without
 * margin) intersect. The penetration depth of a pair of intersecting objects A and B is the length of a point on the
 * boundary of the Minkowski sum (A-B) closest to the origin. The goal of the EPA algorithm is to start with an initial
 * simplex polytope that contains the origin and expend it in order to find the point on the boundary of (A-B) that is
 * closest to the origin. An initial simplex that contains origin has been computed wit GJK algorithm. The EPA Algorithm
 * will extend this simplex polytope to find the correct penetration depth. The implementation of the EPA algorithm is
 * based on the book "Collision Detection in 3D Environments".
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class EPAAlgorithm {

    // Maximum number of support points of the polytope
    private static final int MAX_SUPPORT_POINTS = 100;

    // Maximum number of facets of the polytope
    private static final int MAX_FACETS = 200;

    // Triangle comparison operator
    private TriangleComparison mTriangleComparison;

    // Constructor
    public EPAAlgorithm() {
        mTriangleComparison = new TriangleComparison();
    }

    // Add a triangle face in the candidate triangle heap in the EPA algorithm
    private void addFaceCandidate(TriangleEPA triangle, Queue<TriangleEPA> heap, int[] nbTriangles, float upperBoundSquarePenDepth) {

        // If the closest point of the affine hull of triangle
        // points is internal to the triangle and if the distance
        // of the closest point from the origin is at most the
        // penetration depth upper bound
        if (triangle.isClosestPointInternalToTriangle()
                && triangle.getDistSquare() <= upperBoundSquarePenDepth) {

            // Add the triangle face to the list of candidates
            heap.add(triangle);
            nbTriangles[0]++;
        }
    }

    // Decide if the origin is in the tetrahedron.
    // Return 0 if the origin is in the tetrahedron and return the number (1,2,3 or 4) of
    // the vertex that is wrong if the origin is not in the tetrahedron
    private int isOriginInTetrahedron(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4) {

        // Check vertex 1
        Vector3 normal1 = new Vector3(p2).subtract(p1).cross(new Vector3(p3).subtract(p1));
        if (normal1.dot(p1) > 0.0f == normal1.dot(p4) > 0.0f) {
            return 4;
        }

        // Check vertex 2
        Vector3 normal2 = new Vector3(p4).subtract(p2).cross(new Vector3(p3).subtract(p2));
        if (normal2.dot(p2) > 0.0f == normal2.dot(p1) > 0.0f) {
            return 1;
        }

        // Check vertex 3
        Vector3 normal3 = new Vector3(p4).subtract(p3).cross(new Vector3(p1).subtract(p3));
        if (normal3.dot(p3) > 0.0f == normal3.dot(p2) > 0.0f) {
            return 2;
        }

        // Check vertex 4
        Vector3 normal4 = new Vector3(p2).subtract(p4).cross(new Vector3(p1).subtract(p4));
        if (normal4.dot(p4) > 0.0f == normal4.dot(p3) > 0.0f) {
            return 3;
        }

        // The origin is in the tetrahedron, we return 0
        return 0;
    }

    // Compute the penetration depth with the EPA algorithm.
    // This method computes the penetration depth and contact points between two
    // enlarged objects (with margin) where the original objects (without margin)
    // intersect. An initial simplex that contains origin has been computed with
    // GJK algorithm. The EPA Algorithm will extend this simplex polytope to find
    // the correct penetration depth
    public boolean computePenetrationDepthAndContactPoints(Simplex simplex,
            CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            Vector3 v, ContactPointInfo contactInfo) {

        Vector3[] suppPointsA = new Vector3[MAX_SUPPORT_POINTS];  // Support points of object A in local coordinates
        Vector3[] suppPointsB = new Vector3[MAX_SUPPORT_POINTS];  // Support points of object B in local coordinates
        Vector3[] points = new Vector3[MAX_SUPPORT_POINTS];       // Current points
        TrianglesStore triangleStore = new TrianglesStore();      // Store the triangles
        Queue<TriangleEPA> triangleHeap = new PriorityQueue<>(MAX_FACETS, mTriangleComparison); // Heap that contains the face

        // candidate of the EPA algorithm
        // Transform a point from local space of body 2 to local
        // space of body 1 (the GJK algorithm is done in local space of body 1)
        Transform body2Tobody1 = new Transform(transform1).inverse().multiply(transform2);

        // Matrix that transform a direction from local
        // space of body 1 into local space of body 2
        Matrix3x3 rotation1 = new Matrix3x3();
        transform1.getOrientation().getMatrix(rotation1);
        Matrix3x3 rotation2 = new Matrix3x3();
        transform2.getOrientation().getMatrix(rotation2);
        Matrix3x3 rotateToBody2 = new Matrix3x3(rotation2.transpose()).multiply(rotation1);

        // Get the simplex computed previously by the GJK algorithm
        int nbVertices = simplex.getSimplex(suppPointsA, suppPointsB, points);

        // Compute the tolerance
        float tolerance = Defaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint();

        // Number of triangles in the polytope
        int[] nbTriangles = new int[1];
        nbTriangles[0] = 0;

        // Clear the storing of triangles
        triangleStore.clear();

        // Select an action according to the number of points in the simplex
        // computed with GJK algorithm in order to obtain an initial polytope for
        // The EPA algorithm.
        switch (nbVertices) {
            case 1:
                // Only one point in the simplex (which should be the origin).
                // We have a touching contact with zero penetration depth.
                // We drop that kind of contact. Therefore, we return false
                return false;

            case 2: {
                // The simplex returned by GJK is a line segment d containing the origin.
                // We add two additional support points to construct a hexahedron (two tetrahedron
                // glued together with triangle faces. The idea is to compute three different vectors
                // v1, v2 and v3 that are orthogonal to the segment d. The three vectors are relatively
                // rotated of 120 degree around the d segment. The the three new points to
                // construct the polytope are the three support points in those three directions
                // v1, v2 and v3.

                // Direction of the segment
                Vector3 d = new Vector3(points[1]).subtract(points[0]).normalize();

                // Choose the coordinate axis from the minimal absolute component of the vector d
                int minAxis = new Vector3(d).abs().getMinAxis();

                // Compute sin(60)
                float sin60 = (float) Math.sqrt(3.0f) * 0.5f;

                // Create a rotation quaternion to rotate the vector v1 to get the vectors
                // v2 and v3
                Quaternion rotationQuat = new Quaternion(d.getX() * sin60, d.getY() * sin60, d.getZ() * sin60, 0.5f);

                // Construct the corresponding rotation matrix
                Matrix3x3 rotationMat = new Matrix3x3();
                rotationQuat.getMatrix(rotationMat);

                // Compute the vector v1, v2, v3
                Vector3 v1 = new Vector3(d).cross(new Vector3(minAxis == 0 ? 1.0f : 0.0f, minAxis == 1 ? 1.0f : 0.0f, minAxis == 2 ? 1.0f : 0.0f));
                Vector3 v2 = rotationMat.multiply(v1, new Vector3());
                Vector3 v3 = rotationMat.multiply(v2, new Vector3());

                // Compute the support point in the direction of v1
                suppPointsA[2] = collisionShape1.getLocalSupportPointWithMargin(v1);
                suppPointsB[2] = body2Tobody1.multiply(
                        collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.multiply(new Vector3(v1).invert(), new Vector3())), new Vector3());
                points[2] = new Vector3(suppPointsA[2]).subtract(suppPointsB[2]);

                // Compute the support point in the direction of v2
                suppPointsA[3] = collisionShape1.getLocalSupportPointWithMargin(v2);
                suppPointsB[3] = body2Tobody1.multiply(
                        collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.multiply(new Vector3(v2).invert(), new Vector3())), new Vector3());
                points[3] = new Vector3(suppPointsA[3]).subtract(suppPointsB[3]);

                // Compute the support point in the direction of v3
                suppPointsA[4] = collisionShape1.getLocalSupportPointWithMargin(v3);
                suppPointsB[4] = body2Tobody1.multiply(
                        collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.multiply(new Vector3(v3).invert(), new Vector3())), new Vector3());
                points[4] = new Vector3(suppPointsA[4]).subtract(suppPointsB[4]);

                // Now we have an hexahedron (two tetrahedron glued together). We can simply keep the
                // tetrahedron that contains the origin in order that the initial polytope of the
                // EPA algorithm is a tetrahedron, which is simpler to deal with.
                // If the origin is in the tetrahedron of points 0, 2, 3, 4
                if (isOriginInTetrahedron(points[0], points[2], points[3], points[4]) == 0) {
                    // We use the point 4 instead of point 1 for the initial tetrahedron
                    suppPointsA[1] = suppPointsA[4];
                    suppPointsB[1] = suppPointsB[4];
                    points[1] = points[4];
                } // If the origin is in the tetrahedron of points 1, 2, 3, 4
                else if (isOriginInTetrahedron(points[1], points[2], points[3], points[4]) == 0) {
                    // We use the point 4 instead of point 0 for the initial tetrahedron
                    suppPointsA[0] = suppPointsA[4];
                    suppPointsB[0] = suppPointsB[4];
                    points[0] = points[4];
                } else {
                    // The origin is not in the initial polytope
                    return false;
                }

                // The polytope contains now 4 vertices
                nbVertices = 4;
            }
            case 4: {
                // The simplex computed by the GJK algorithm is a tetrahedron. Here we check
                // if this tetrahedron contains the origin. If it is the case, we keep it and
                // otherwise we remove the wrong vertex of the tetrahedron and go in the case
                // where the GJK algorithm compute a simplex of three vertices.

                // Check if the tetrahedron contains the origin (or wich is the wrong vertex otherwise)
                int badVertex = isOriginInTetrahedron(points[0], points[1], points[2], points[3]);

                // If the origin is in the tetrahedron
                if (badVertex == 0) {
                    // The tetrahedron is a correct initial polytope for the EPA algorithm.
                    // Therefore, we construct the tetrahedron.

                    // Comstruct the 4 triangle faces of the tetrahedron
                    TriangleEPA face0 = triangleStore.newTriangle(points, 0, 1, 2);
                    TriangleEPA face1 = triangleStore.newTriangle(points, 0, 3, 1);
                    TriangleEPA face2 = triangleStore.newTriangle(points, 0, 2, 3);
                    TriangleEPA face3 = triangleStore.newTriangle(points, 1, 3, 2);

                    // If the constructed tetrahedron is not correct
                    if (!((face0 != null) && (face1 != null) && (face2 != null) && (face3 != null)
                            && face0.getDistSquare() > 0.0f && face1.getDistSquare() > 0.0f
                            && face2.getDistSquare() > 0.0f && face3.getDistSquare() > 0.0f)) {
                        return false;
                    }

                    // Associate the edges of neighbouring triangle faces
                    Utils.link(new EdgeEPA(face0, 0), new EdgeEPA(face1, 2));
                    Utils.link(new EdgeEPA(face0, 1), new EdgeEPA(face3, 2));
                    Utils.link(new EdgeEPA(face0, 2), new EdgeEPA(face2, 0));
                    Utils.link(new EdgeEPA(face1, 0), new EdgeEPA(face2, 2));
                    Utils.link(new EdgeEPA(face1, 1), new EdgeEPA(face3, 0));
                    Utils.link(new EdgeEPA(face2, 1), new EdgeEPA(face3, 1));

                    // Add the triangle faces in the candidate heap
                    addFaceCandidate(face0, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);
                    addFaceCandidate(face1, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);
                    addFaceCandidate(face2, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);
                    addFaceCandidate(face3, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);

                    break;
                }

                // If the tetrahedron contains a wrong vertex (the origin is not inside the tetrahedron)
                if (badVertex < 4) {

                    // Replace the wrong vertex with the point 5 (if it exists)
                    suppPointsA[badVertex - 1] = suppPointsA[4];
                    suppPointsB[badVertex - 1] = suppPointsB[4];
                    points[badVertex - 1] = points[4];
                }

                // We have removed the wrong vertex
                nbVertices = 3;
            }
            case 3: {
                // The GJK algorithm returned a triangle that contains the origin.
                // We need two new vertices to obtain a hexahedron. The two new vertices
                // are the support points in the "n" and "-n" direction where "n" is the
                // normal of the triangle.

                // Compute the normal of the triangle
                Vector3 v1 = new Vector3(points[1]).subtract(points[0]);
                Vector3 v2 = new Vector3(points[2]).subtract(points[0]);
                Vector3 n = new Vector3(v1).cross(v2);

                // Compute the two new vertices to obtain a hexahedron
                suppPointsA[3] = collisionShape1.getLocalSupportPointWithMargin(n);
                suppPointsB[3] = body2Tobody1.multiply(
                        collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.multiply(new Vector3(n).invert(), new Vector3())), new Vector3());
                points[3] = new Vector3(suppPointsA[3]).subtract(suppPointsB[3]);
                suppPointsA[4] = collisionShape1.getLocalSupportPointWithMargin(new Vector3(n).invert());
                suppPointsB[4] = body2Tobody1.multiply(
                        collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.multiply(n, new Vector3())), new Vector3());
                points[4] = new Vector3(suppPointsA[4]).subtract(suppPointsB[4]);

                // Construct the triangle faces
                TriangleEPA face0 = triangleStore.newTriangle(points, 0, 1, 3);
                TriangleEPA face1 = triangleStore.newTriangle(points, 1, 2, 3);
                TriangleEPA face2 = triangleStore.newTriangle(points, 2, 0, 3);
                TriangleEPA face3 = triangleStore.newTriangle(points, 0, 2, 4);
                TriangleEPA face4 = triangleStore.newTriangle(points, 2, 1, 4);
                TriangleEPA face5 = triangleStore.newTriangle(points, 1, 0, 4);

                // If the polytope hasn't been correctly constructed
                if (!((face0 != null) && (face1 != null) && (face2 != null) && (face3 != null)
                        && (face4 != null) && (face5 != null)
                        && face0.getDistSquare() > 0.0f && face1.getDistSquare() > 0.0f
                        && face2.getDistSquare() > 0.0f && face3.getDistSquare() > 0.0f
                        && face4.getDistSquare() > 0.0f && face5.getDistSquare() > 0.0f)) {
                    return false;
                }

                // Associate the edges of neighbouring faces
                Utils.link(new EdgeEPA(face0, 1), new EdgeEPA(face1, 2));
                Utils.link(new EdgeEPA(face1, 1), new EdgeEPA(face2, 2));
                Utils.link(new EdgeEPA(face2, 1), new EdgeEPA(face0, 2));
                Utils.link(new EdgeEPA(face0, 0), new EdgeEPA(face5, 0));
                Utils.link(new EdgeEPA(face1, 0), new EdgeEPA(face4, 0));
                Utils.link(new EdgeEPA(face2, 0), new EdgeEPA(face3, 0));
                Utils.link(new EdgeEPA(face3, 1), new EdgeEPA(face4, 2));
                Utils.link(new EdgeEPA(face4, 1), new EdgeEPA(face5, 2));
                Utils.link(new EdgeEPA(face5, 1), new EdgeEPA(face3, 2));

                // Add the candidate faces in the heap
                addFaceCandidate(face0, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);
                addFaceCandidate(face1, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);
                addFaceCandidate(face2, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);
                addFaceCandidate(face3, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);
                addFaceCandidate(face4, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);
                addFaceCandidate(face5, triangleHeap, nbTriangles, Defaults.DECIMAL_LARGEST);

                nbVertices = 5;
            }
            break;
        }

        // At this point, we have a polytope that contains the origin. Therefore, we
        // can run the EPA algorithm.
        if (nbTriangles[0] == 0) {
            return false;
        }

        TriangleEPA triangle = null;
        float upperBoundSquarePenDepth = Defaults.DECIMAL_LARGEST;

        do {
            triangle = triangleHeap.remove();

            // Get the next candidate face (the face closest to the origin)
            nbTriangles[0]--;

            // If the candidate face in the heap is not obsolete
            if (!triangle.getIsObsolete()) {
                // If we have reached the maximum number of support points
                if (nbVertices == MAX_SUPPORT_POINTS) {
                    assert (false);
                    break;
                }

                // Compute the support point of the Minkowski
                // difference (A-B) in the closest point direction
                suppPointsA[nbVertices] = collisionShape1.getLocalSupportPointWithMargin(triangle.getClosestPoint());
                suppPointsB[nbVertices] = body2Tobody1.multiply(
                        collisionShape2.getLocalSupportPointWithMargin(
                                rotateToBody2.multiply(new Vector3(triangle.getClosestPoint()).invert(), new Vector3())), new Vector3());
                points[nbVertices] = new Vector3(suppPointsA[nbVertices]).subtract(suppPointsB[nbVertices]);

                int indexNewVertex = nbVertices;
                nbVertices++;

                // Update the upper bound of the penetration depth
                float wDotv = points[indexNewVertex].dot(triangle.getClosestPoint());
                assert (wDotv > 0.0f);
                float wDotVSquare = wDotv * wDotv / triangle.getDistSquare();
                if (wDotVSquare < upperBoundSquarePenDepth) {
                    upperBoundSquarePenDepth = wDotVSquare;
                }

                // Compute the error
                float error = wDotv - triangle.getDistSquare();
                if (error <= Math.max(tolerance, GJKAlgorithm.REL_ERROR_SQUARE * wDotv)
                        || points[indexNewVertex] == points[triangle.operatorSquareBrackets(0)]
                        || points[indexNewVertex] == points[triangle.operatorSquareBrackets(1)]
                        || points[indexNewVertex] == points[triangle.operatorSquareBrackets(2)]) {
                    break;
                }

                // Now, we compute the silhouette cast by the new vertex. The current triangle
                // face will not be in the convex hull. We start the local recursive silhouette
                // algorithm from the current triangle face.
                int i = triangleStore.getNbTriangles();
                if (!triangle.computeSilhouette(points, indexNewVertex, triangleStore)) {
                    break;
                }

                // Add all the new triangle faces computed with the silhouette algorithm
                // to the candidates list of faces of the current polytope
                while (i != triangleStore.getNbTriangles()) {
                    TriangleEPA newTriangle = triangleStore.operatorSquareBrackets(i);
                    addFaceCandidate(newTriangle, triangleHeap, nbTriangles, upperBoundSquarePenDepth);
                    i++;
                }
            }
        } while (nbTriangles[0] > 0 && triangleHeap.element().getDistSquare() <= upperBoundSquarePenDepth);

        // Compute the contact info
        transform1.getOrientation().getMatrix(rotation1);
        v = rotation1.multiply(triangle.getClosestPoint(), new Vector3());
        Vector3 pALocal = triangle.computeClosestPointOfObject(suppPointsA);
        Vector3 pBLocal = new Transform(body2Tobody1).inverse().multiply(triangle.computeClosestPointOfObject(suppPointsB), new Vector3());
        Vector3 normal = new Vector3(v).normalize();
        float penetrationDepth = v.length();
        assert (penetrationDepth > 0.0f);

        // Create the contact info object
        contactInfo.normal = normal;
        contactInfo.penetrationDepth = penetrationDepth;
        contactInfo.localPoint1 = pALocal;
        contactInfo.localPoint2 = pBLocal;

        return true;
    }

}
