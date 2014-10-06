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
package net.smert.jreactphysics3d.collision.narrowphase.EPA;

import java.util.PriorityQueue;
import java.util.Queue;
import net.smert.jreactphysics3d.collision.narrowphase.GJK.GJKAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.GJK.Simplex;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
import net.smert.jreactphysics3d.mathematics.Mathematics;
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

    // Maximum number of facets of the polytope
    private final static int MAX_FACETS = 200;

    // Maximum number of support points of the polytope
    private final static int MAX_SUPPORT_POINTS = 100;

    // Matrix that transform a direction from local space of body 1 into local space of body 2
    private final Matrix3x3 rotateToBody2;

    // Transform a point from local space of body 2 to local space of body 1
    private final Transform body2ToBody1;

    // Triangle comparison operator
    private final TriangleComparison triangleComparison;

    // Constructor
    public EPAAlgorithm() {
        rotateToBody2 = new Matrix3x3();
        body2ToBody1 = new Transform();
        triangleComparison = new TriangleComparison();
    }

    // Add a triangle face in the candidate triangle heap in the EPA algorithm
    private void addFaceCandidate(TriangleEPA triangle, Queue<TriangleEPA> heap,
            int[] numTriangles, float upperBoundSquarePenDepth) {

        // If the closest point of the affine hull of triangle
        // points is internal to the triangle and if the distance
        // of the closest point from the origin is at most the
        // penetration depth upper bound
        if (triangle.isClosestPointInternalToTriangle() && triangle.getDistanceSquare() <= upperBoundSquarePenDepth) {

            // Add the triangle face to the list of candidates
            heap.add(triangle);
            numTriangles[0]++;
        }
    }

    // Decide if the origin is in the tetrahedron.
    // Return 0 if the origin is in the tetrahedron and return the number (1,2,3 or 4) of
    // the vertex that is wrong if the origin is not in the tetrahedron
    private int isOriginInTetrahedron(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4) {

        Vector3 vector1 = new Vector3(p2);
        Vector3 vector2 = new Vector3(p3);

        // Check vertex 1
        Vector3 normal1 = vector1.subtract(p1).cross(vector2.subtract(p1));
        if (normal1.dot(p1) > 0.0f == normal1.dot(p4) > 0.0f) {
            return 4;
        }

        // Check vertex 2
        Vector3 normal2 = vector1.set(p4).subtract(p2).cross(vector2.set(p3).subtract(p2));
        if (normal2.dot(p2) > 0.0f == normal2.dot(p1) > 0.0f) {
            return 1;
        }

        // Check vertex 3
        Vector3 normal3 = vector1.set(p4).subtract(p3).cross(vector2.set(p1).subtract(p3));
        if (normal3.dot(p3) > 0.0f == normal3.dot(p2) > 0.0f) {
            return 2;
        }

        // Check vertex 4
        Vector3 normal4 = vector1.set(p2).subtract(p4).cross(vector2.set(p1).subtract(p4));
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
    public boolean computePenetrationDepthAndContactPoints(
            CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            ContactPointInfo contactInfo, Simplex simplex) {

        final Matrix3x3 tempRotation1 = new Matrix3x3();
        final Matrix3x3 tempRotation2 = new Matrix3x3();
        final Queue<TriangleEPA> triangleHeap = new PriorityQueue<>(MAX_FACETS, triangleComparison); // Heap that contains the face
        final TrianglesStore triangleStore = new TrianglesStore();      // Store the triangles
        final Vector3[] minkDiffs = new Vector3[MAX_SUPPORT_POINTS];    // Support point of Minkowski difference supportA-supportB
        final Vector3[] supportAs = new Vector3[MAX_SUPPORT_POINTS];    // Support points of object A in local coordinates
        final Vector3[] supportBs = new Vector3[MAX_SUPPORT_POINTS];    // Support points of object B in local coordinates

        // candidate of the EPA algorithm
        // Transform a point from local space of body 2 to local
        // space of body 1 (the GJK algorithm is done in local space of body 1)
        body2ToBody1.set(transform1).inverse().multiply(transform2);

        // Matrix that transform a direction from local
        // space of body 1 into local space of body 2
        transform1.getOrientation().getMatrix(tempRotation1);
        transform2.getOrientation().getMatrix(tempRotation2);
        rotateToBody2.set(tempRotation2.transpose()).multiply(tempRotation1);

        // Get the simplex computed previously by the GJK algorithm
        int numVertices = simplex.getSimplex(supportAs, supportBs, minkDiffs);

        // Compute the tolerance
        float tolerance = Defaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint();

        // Number of triangles in the polytope
        int[] numTriangles = new int[1];
        numTriangles[0] = 0;

        // Clear the storing of triangles
        triangleStore.clear();

        // Select an action according to the number of points in the simplex
        // computed with GJK algorithm in order to obtain an initial polytope for
        // The EPA algorithm.
        switch (numVertices) {
            case 1:
                // Only one point in the simplex (which should be the origin).
                // We have a touching contact with zero penetration depth.
                // We drop that kind of contact. Therefore, we return false
                return false;

            case 2: {
                // The simplex returned by GJK is a line segment "dir" containing the origin.
                // We add two additional support points to construct a hexahedron (two tetrahedron
                // glued together with triangle faces. The idea is to compute three different vectors
                // v1, v2 and v3 that are orthogonal to the segment "dir". The three vectors are relatively
                // rotated of 120 degree around the "dir" segment. The the three new points to
                // construct the polytope are the three support points in those three directions
                // v1, v2 and v3.

                // Direction of the segment
                Vector3 dir = new Vector3(minkDiffs[1]).subtract(minkDiffs[0]).normalize();

                // Choose the coordinate axis from the minimal absolute component of the vector d
                int minAxis = new Vector3(dir).abs().getMinAxis();

                // Compute sin(60)
                float sin60 = Mathematics.Sqrt(3.0f) * 0.5f;

                // Create a rotation quaternion to rotate the vector v1 to get the vectors
                // v2 and v3
                Quaternion rotationQuat = new Quaternion(dir.getX() * sin60, dir.getY() * sin60, dir.getZ() * sin60, 0.5f);

                // Construct the corresponding rotation matrix
                rotationQuat.getMatrix(tempRotation1);

                // Compute the vector v1, v2, v3
                Vector3 dir1 = new Vector3(dir).cross(new Vector3(minAxis == 0 ? 1.0f : 0.0f, minAxis == 1 ? 1.0f : 0.0f, minAxis == 2 ? 1.0f : 0.0f));
                Vector3 dir2 = tempRotation1.multiply(dir1, new Vector3());
                Vector3 dir3 = tempRotation1.multiply(dir2, new Vector3());

                // Compute the support point in the direction of v1
                supportAs[2] = collisionShape1.getLocalSupportPointWithMargin(dir1, new Vector3());
                dir1.set(rotateToBody2.multiply(dir1.invert(), new Vector3()));
                supportBs[2] = collisionShape2.getLocalSupportPointWithMargin(dir1, new Vector3());
                supportBs[2] = body2ToBody1.multiply(supportBs[2], new Vector3());
                minkDiffs[2] = new Vector3(supportAs[2]).subtract(supportBs[2]);

                // Compute the support point in the direction of v2
                supportAs[3] = collisionShape1.getLocalSupportPointWithMargin(dir2, new Vector3());
                dir2.set(rotateToBody2.multiply(dir2.invert(), new Vector3()));
                supportBs[3] = collisionShape2.getLocalSupportPointWithMargin(dir2, new Vector3());
                supportBs[3] = body2ToBody1.multiply(supportBs[3], new Vector3());
                minkDiffs[3] = new Vector3(supportAs[3]).subtract(supportBs[3]);

                // Compute the support point in the direction of v3
                supportAs[4] = collisionShape1.getLocalSupportPointWithMargin(dir3, new Vector3());
                dir3.set(rotateToBody2.multiply(dir3.invert(), new Vector3()));
                supportBs[4] = collisionShape2.getLocalSupportPointWithMargin(dir3, new Vector3());
                supportBs[4] = body2ToBody1.multiply(supportBs[4], new Vector3());
                minkDiffs[4] = new Vector3(supportAs[4]).subtract(supportBs[4]);

                // Now we have an hexahedron (two tetrahedron glued together). We can simply keep the
                // tetrahedron that contains the origin in order that the initial polytope of the
                // EPA algorithm is a tetrahedron, which is simpler to deal with.
                // If the origin is in the tetrahedron of points 0, 2, 3, 4
                if (isOriginInTetrahedron(minkDiffs[0], minkDiffs[2], minkDiffs[3], minkDiffs[4]) == 0) {
                    // We use the point 4 instead of point 1 for the initial tetrahedron
                    supportAs[1] = supportAs[4];
                    supportBs[1] = supportBs[4];
                    minkDiffs[1] = minkDiffs[4];
                } // If the origin is in the tetrahedron of points 1, 2, 3, 4
                else if (isOriginInTetrahedron(minkDiffs[1], minkDiffs[2], minkDiffs[3], minkDiffs[4]) == 0) {
                    // We use the point 4 instead of point 0 for the initial tetrahedron
                    supportAs[0] = supportAs[4];
                    supportBs[0] = supportBs[4];
                    minkDiffs[0] = minkDiffs[4];
                } else {
                    // The origin is not in the initial polytope
                    return false;
                }

                // The polytope contains now 4 vertices
                numVertices = 4;
            }
            case 4: {
                // The simplex computed by the GJK algorithm is a tetrahedron. Here we check
                // if this tetrahedron contains the origin. If it is the case, we keep it and
                // otherwise we remove the wrong vertex of the tetrahedron and go in the case
                // where the GJK algorithm compute a simplex of three vertices.

                // Check if the tetrahedron contains the origin (or wich is the wrong vertex otherwise)
                int badVertex = isOriginInTetrahedron(minkDiffs[0], minkDiffs[1], minkDiffs[2], minkDiffs[3]);

                // If the origin is in the tetrahedron
                if (badVertex == 0) {
                    // The tetrahedron is a correct initial polytope for the EPA algorithm.
                    // Therefore, we construct the tetrahedron.

                    // Comstruct the 4 triangle faces of the tetrahedron
                    TriangleEPA face0 = triangleStore.newTriangle(minkDiffs, 0, 1, 2);
                    TriangleEPA face1 = triangleStore.newTriangle(minkDiffs, 0, 3, 1);
                    TriangleEPA face2 = triangleStore.newTriangle(minkDiffs, 0, 2, 3);
                    TriangleEPA face3 = triangleStore.newTriangle(minkDiffs, 1, 3, 2);

                    // If the constructed tetrahedron is not correct
                    if (!((face0 != null) && (face1 != null) && (face2 != null) && (face3 != null)
                            && face0.getDistanceSquare() > 0.0f && face1.getDistanceSquare() > 0.0f
                            && face2.getDistanceSquare() > 0.0f && face3.getDistanceSquare() > 0.0f)) {
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
                    addFaceCandidate(face0, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);
                    addFaceCandidate(face1, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);
                    addFaceCandidate(face2, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);
                    addFaceCandidate(face3, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);

                    break;
                }

                // If the tetrahedron contains a wrong vertex (the origin is not inside the tetrahedron)
                if (badVertex < 4) {

                    // Replace the wrong vertex with the point 5 (if it exists)
                    supportAs[badVertex - 1] = supportAs[4];
                    supportBs[badVertex - 1] = supportBs[4];
                    minkDiffs[badVertex - 1] = minkDiffs[4];
                }

                // We have removed the wrong vertex
                numVertices = 3;
            }
//            case 3: {
//                // The GJK algorithm returned a triangle that contains the origin.
//                // We need two new vertices to obtain a hexahedron. The two new vertices
//                // are the support points in the "dir1" and "-dir1" direction where "dir1" is the
//                // normal of the triangle.
//
//                // Compute the normal of the triangle
//                Vector3 v1 = new Vector3(minkDiffs[1]).subtract(minkDiffs[0]);
//                Vector3 v2 = new Vector3(minkDiffs[2]).subtract(minkDiffs[0]);
//                Vector3 dir1 = new Vector3(v1).cross(v2);
//                Vector3 dir2 = new Vector3(dir1).invert();
//                Vector3 dir3;
//
//                // Compute the two new vertices to obtain a hexahedron
//                supportAs[3] = collisionShape1.getLocalSupportPointWithMargin(dir1, new Vector3());
//                dir3 = rotateToBody2.multiply(dir2, new Vector3());
//                supportBs[3] = collisionShape2.getLocalSupportPointWithMargin(dir3, new Vector3());
//                supportBs[3] = body2ToBody1.multiply(supportBs[3], new Vector3());
//                minkDiffs[3] = new Vector3(supportAs[3]).subtract(supportBs[3]);
//
//                supportAs[4] = collisionShape1.getLocalSupportPointWithMargin(dir2, new Vector3());
//                dir3 = rotateToBody2.multiply(dir1, new Vector3());
//                supportBs[4] = collisionShape2.getLocalSupportPointWithMargin(dir3, new Vector3());
//                supportBs[4] = body2ToBody1.multiply(supportBs[4], new Vector3());
//                minkDiffs[4] = new Vector3(supportAs[4]).subtract(supportBs[4]);
//
//                // Construct the triangle faces
//                TriangleEPA face0 = triangleStore.newTriangle(minkDiffs, 0, 1, 3);
//                TriangleEPA face1 = triangleStore.newTriangle(minkDiffs, 1, 2, 3);
//                TriangleEPA face2 = triangleStore.newTriangle(minkDiffs, 2, 0, 3);
//                TriangleEPA face3 = triangleStore.newTriangle(minkDiffs, 0, 2, 4);
//                TriangleEPA face4 = triangleStore.newTriangle(minkDiffs, 2, 1, 4);
//                TriangleEPA face5 = triangleStore.newTriangle(minkDiffs, 1, 0, 4);
//
//                // If the polytope hasn't been correctly constructed
//                if (!((face0 != null) && (face1 != null) && (face2 != null) && (face3 != null)
//                        && (face4 != null) && (face5 != null)
//                        && face0.getDistanceSquare() > 0.0f && face1.getDistanceSquare() > 0.0f
//                        && face2.getDistanceSquare() > 0.0f && face3.getDistanceSquare() > 0.0f
//                        && face4.getDistanceSquare() > 0.0f && face5.getDistanceSquare() > 0.0f)) {
//                    return false;
//                }
//
//                // Associate the edges of neighbouring faces
//                Utils.link(new EdgeEPA(face0, 1), new EdgeEPA(face1, 2));
//                Utils.link(new EdgeEPA(face1, 1), new EdgeEPA(face2, 2));
//                Utils.link(new EdgeEPA(face2, 1), new EdgeEPA(face0, 2));
//                Utils.link(new EdgeEPA(face0, 0), new EdgeEPA(face5, 0));
//                Utils.link(new EdgeEPA(face1, 0), new EdgeEPA(face4, 0));
//                Utils.link(new EdgeEPA(face2, 0), new EdgeEPA(face3, 0));
//                Utils.link(new EdgeEPA(face3, 1), new EdgeEPA(face4, 2));
//                Utils.link(new EdgeEPA(face4, 1), new EdgeEPA(face5, 2));
//                Utils.link(new EdgeEPA(face5, 1), new EdgeEPA(face3, 2));
//
//                // Add the candidate faces in the heap
//                addFaceCandidate(face0, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);
//                addFaceCandidate(face1, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);
//                addFaceCandidate(face2, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);
//                addFaceCandidate(face3, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);
//                addFaceCandidate(face4, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);
//                addFaceCandidate(face5, triangleHeap, numTriangles, Defaults.DECIMAL_LARGEST);
//
//                numVertices = 5;
//            }
            break;
        }

        // At this point, we have a polytope that contains the origin. Therefore, we
        // can run the EPA algorithm.
        if (numTriangles[0] == 0) {
            return false;
        }

        TriangleEPA triangle;
        float upperBoundSquarePenDepth = Defaults.DECIMAL_LARGEST;

        do {
            triangle = triangleHeap.remove();

            // Get the next candidate face (the face closest to the origin)
            numTriangles[0]--;

            // If the candidate face in the heap is not obsolete
            if (!triangle.getIsObsolete()) {
                // If we have reached the maximum number of support points
                if (numVertices == MAX_SUPPORT_POINTS) {
                    assert (false);
                    break;
                }

                // Compute the support point of the Minkowski
                // difference (A-B) in the closest point direction
                Vector3 dir = new Vector3(triangle.getClosestPoint());
                supportAs[numVertices] = collisionShape1.getLocalSupportPointWithMargin(dir, new Vector3());
                dir.set(rotateToBody2.multiply(dir.invert(), new Vector3()));
                supportBs[numVertices] = collisionShape2.getLocalSupportPointWithMargin(dir, new Vector3());
                supportBs[numVertices] = body2ToBody1.multiply(supportBs[numVertices], new Vector3());
                minkDiffs[numVertices] = new Vector3(supportAs[numVertices]).subtract(supportBs[numVertices]);

                int indexNewVertex = numVertices;
                numVertices++;

                // Update the upper bound of the penetration depth
                float minkDiffDotDirection = minkDiffs[indexNewVertex].dot(triangle.getClosestPoint());
                assert (minkDiffDotDirection > 0.0f);
                float minkDiffDotDirectionSquare = minkDiffDotDirection * minkDiffDotDirection / triangle.getDistanceSquare();
                if (minkDiffDotDirectionSquare < upperBoundSquarePenDepth) {
                    upperBoundSquarePenDepth = minkDiffDotDirectionSquare;
                }

                // Compute the error
                float error = minkDiffDotDirection - triangle.getDistanceSquare();
                if (error <= Math.max(tolerance, GJKAlgorithm.REL_ERROR_SQUARE * minkDiffDotDirection)
                        || minkDiffs[indexNewVertex].equals(minkDiffs[triangle.getIndexVertex(0)])
                        || minkDiffs[indexNewVertex].equals(minkDiffs[triangle.getIndexVertex(1)])
                        || minkDiffs[indexNewVertex].equals(minkDiffs[triangle.getIndexVertex(2)])) {
                    break;
                }

                // Now, we compute the silhouette cast by the new vertex. The current triangle
                // face will not be in the convex hull. We start the local recursive silhouette
                // algorithm from the current triangle face.
                int index = triangleStore.getNumTriangles();
                if (!triangle.computeSilhouette(minkDiffs, indexNewVertex, triangleStore)) {
                    break;
                }

                // Add all the new triangle faces computed with the silhouette algorithm
                // to the candidates list of faces of the current polytope
                while (index != triangleStore.getNumTriangles()) {
                    TriangleEPA newTriangle = triangleStore.get(index);
                    addFaceCandidate(newTriangle, triangleHeap, numTriangles, upperBoundSquarePenDepth);
                    index++;
                }
            }

        } while (numTriangles[0] > 0 && triangleHeap.element().getDistanceSquare() <= upperBoundSquarePenDepth);

        assert (triangle != null);

        // Compute the contact info
        transform1.getOrientation().getMatrix(tempRotation1);
        Vector3 normal = tempRotation1.multiply(triangle.getClosestPoint(), new Vector3());
        float penetrationDepth = normal.length();
        Vector3 pointA = triangle.computeClosestPointOfObject(supportAs);
        Vector3 pointB = triangle.computeClosestPointOfObject(supportBs);
        pointB.set(body2ToBody1.inverse().multiply(pointB, new Vector3()));

        assert (penetrationDepth > 0.0f);

        // Create the contact info object
        contactInfo.setCollisionData(normal.normalize(), penetrationDepth, pointA, pointB);

        return true;
    }

}
