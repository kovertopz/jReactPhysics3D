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
package net.smert.jreactphysics3d.collision.shapes;

import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a capsule collision shape that is defined around the Y axis. A capsule shape can be seen as the
 * convex hull of two spheres. The capsule shape is defined by its radius (radius of the two spheres of the capsule) and
 * its height (distance between the centers of the two spheres). This collision shape does not have an explicit object
 * margin distance. The margin is implicitly the radius and height of the shape. Therefore, no need to specify an object
 * margin for a capsule shape.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CapsuleShape extends CollisionShape {

    // Half height of the capsule (height = distance between the centers of the two spheres)
    private final float halfHeight;

    // Radius of the two spheres of the capsule
    private final float radius;

    // Constructor
    public CapsuleShape(float radius, float height, float margin) {

        // TODO: Should radius really be the margin for a capsule? Seems like a bug.
        super(CollisionShapeType.CAPSULE, radius + margin);
        assert (height > 0.0f);
        assert (radius > 0.0f);
        halfHeight = height * 0.5f;
        this.radius = radius;
    }

    // Copy-constructor
    public CapsuleShape(CapsuleShape shape) {
        super(shape);
        halfHeight = shape.halfHeight;
        radius = shape.radius;
    }

    // Get the radius of the capsule
    public float getRadius() {
        return radius;
    }

    // Return the height of the capsule
    public float getHeight() {
        return halfHeight + halfHeight;
    }

    // Test equality between two capsule shapes
    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        CapsuleShape otherShape = (CapsuleShape) otherCollisionShape;
        return (radius == otherShape.radius && halfHeight == otherShape.halfHeight);
    }

    // Return a local support point in a given direction with the object margin.
    // A capsule is the convex hull of two spheres S1 and S2. The support point in the direction "d"
    // of the convex hull of a set of convex objects is the support point "p" in the set of all
    // support points from all the convex objects with the maximum dot product with the direction "d".
    // Therefore, in this method, we compute the support points of both top and bottom spheres of
    // the capsule and return the point with the maximum dot product with the direction vector. Note
    // that the object margin is implicitly the radius and height of the capsule.
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction, Vector3 supportPoint) {

        // If the direction vector is not the zero vector
        if (direction.lengthSquare() >= Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {

            Vector3 unitDirection = new Vector3(direction).normalize();

            // Support point top sphere
            Vector3 centerTopSphere = new Vector3(0.0f, halfHeight, 0.0f);
            Vector3 topSpherePoint = new Vector3(centerTopSphere).add(unitDirection).multiply(radius + margin);
            float dotProductTop = topSpherePoint.dot(direction);

            // Support point bottom sphere
            Vector3 centerBottomSphere = new Vector3(0.0f, -halfHeight, 0.0f);
            Vector3 bottomSpherePoint = new Vector3(centerBottomSphere).add(unitDirection).multiply(radius + margin);
            float dotProductBottom = bottomSpherePoint.dot(direction);

            // Return the point with the maximum dot product
            if (dotProductTop > dotProductBottom) {
                return supportPoint.set(topSpherePoint);
            } else {
                return supportPoint.set(bottomSpherePoint);
            }
        }

        // If the direction vector is the zero vector we return a point on the
        // boundary of the capsule
        return supportPoint.set(0.0f, radius + margin, 0.0f);
    }

    // Return a local support point in a given direction without the object margin.
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction, Vector3 supportPoint) {

        // If the dot product of the direction and the local Y axis (dotProduct = direction.y)
        // is positive
        if (direction.getY() > 0.0f) {

            // Return the top sphere center point
            return supportPoint.set(0.0f, halfHeight, 0.0f);
        }

        // Return the bottom sphere center point
        return supportPoint.set(0.0f, -halfHeight, 0.0f);
    }

    // Return the local inertia tensor of the capsule
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {

        // The inertia tensor formula for a capsule can be found in : Game Engine Gems, Volume 1
        float height = halfHeight + halfHeight;
        float radiusSquare = radius * radius;
        float heightSquare = height * height;
        float radiusSquareDouble = radiusSquare + radiusSquare;
        float factor1 = 2.0f * radius / (4.0f * radius + 3.0f * height);
        float factor2 = 3.0f * height / (4.0f * radius + 3.0f * height);
        float sum1 = 0.4f * radiusSquareDouble;
        float sum2 = 0.75f * height * radius + 0.5f * heightSquare;
        float sum3 = 0.25f * radiusSquare + 1.0f / 12.0f * heightSquare;
        float IxxAndzz = factor1 * mass * (sum1 + sum2) + factor2 * mass * sum3;
        float Iyy = factor1 * mass * sum1 + factor2 * mass * 0.25f * radiusSquareDouble;
        tensor.set(IxxAndzz, 0.0f, 0.0f,
                0.0f, Iyy, 0.0f,
                0.0f, 0.0f, IxxAndzz);
    }

    // Return the local bounds of the shape in x, y and z directions
    // This method is used to compute the AABB of the box
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {

        // Maximum bounds
        max.setX(radius + margin);
        max.setY(halfHeight + radius + margin);
        max.setZ(max.getX());

        // Minimum bounds
        min.setX(-max.getX());
        min.setY(-max.getY());
        min.setZ(min.getX());
    }

    @Override
    public CollisionShape clone() {
        return new CapsuleShape(this);
    }

}
