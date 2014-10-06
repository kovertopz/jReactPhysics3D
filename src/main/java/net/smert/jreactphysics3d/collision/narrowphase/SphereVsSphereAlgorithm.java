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
            contactInfo.setCollisionData(vectorBetweenCenters.normalize(), penetrationDepth, intersectionOnBody1, intersectionOnBody2);

            return true;
        }

        return false;
    }

}
