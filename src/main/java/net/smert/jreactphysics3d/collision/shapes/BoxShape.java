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

import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a 3D box shape. Those axis are unit length. The three extents are half-widths of the box along
 * the three axis x, y, z local axis. The "transform" of the corresponding rigid body will give an orientation and a
 * position to the box. This collision shape uses an extra margin distance around it for collision detection purpose.
 * The default margin is 4cm (if your units are meters, which is recommended). In case, you want to simulate small
 * objects (smaller than the margin distance), you might want to reduce the margin by specifying your own margin
 * distance using the "margin" parameter in the constructor of the box shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BoxShape extends CollisionShape {

    // Extent sizes of the box in the x, y and z direction
    private final Vector3 extent;

    // Constructor
    public BoxShape(Vector3 extent, float margin) {
        super(CollisionShapeType.BOX, margin);
        assert (extent.getX() > 0.0f && extent.getX() > margin);
        assert (extent.getY() > 0.0f && extent.getY() > margin);
        assert (extent.getZ() > 0.0f && extent.getZ() > margin);
        this.extent = new Vector3(extent);
    }

    // Copy-constructor
    public BoxShape(BoxShape shape) {
        super(shape);
        extent = new Vector3(shape.extent);
    }

    // Return the extents of the box
    public Vector3 getExtent() {
        return extent;
    }

    // Test equality between two box shapes
    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        BoxShape otherShape = (BoxShape) otherCollisionShape;
        return extent.equals(otherShape.extent);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction, Vector3 supportPoint) {
        return supportPoint.set(
                direction.getX() < 0.0f ? -extent.getX() - margin : extent.getX() + margin,
                direction.getY() < 0.0f ? -extent.getY() - margin : extent.getY() + margin,
                direction.getZ() < 0.0f ? -extent.getZ() - margin : extent.getZ() + margin);
    }

    // Return a local support point in a given direction without the objec margin
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction, Vector3 supportPoint) {
        return supportPoint.set(
                direction.getX() < 0.0f ? -extent.getX() : extent.getX(),
                direction.getY() < 0.0f ? -extent.getY() : extent.getY(),
                direction.getZ() < 0.0f ? -extent.getZ() : extent.getZ());
    }

    // Return the local inertia tensor of the collision shape
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        float factor = (1.0f / 3.0f) * mass;
        float xSquare = extent.getX() * extent.getX();
        float ySquare = extent.getY() * extent.getY();
        float zSquare = extent.getZ() * extent.getZ();
        tensor.set(factor * (ySquare + zSquare), 0.0f, 0.0f,
                0.0f, factor * (xSquare + zSquare), 0.0f,
                0.0f, 0.0f, factor * (xSquare + ySquare));
    }

    // Return the local bounds of the shape in x, y and z directions
    // This method is used to compute the AABB of the box
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {

        // Maximum bounds
        max.set(extent.getX(), extent.getY(), extent.getZ());
        max.add(new Vector3(margin, margin, margin));

        // Minimum bounds
        min.set(-max.getX(), -max.getY(), -max.getZ());
    }

    @Override
    public CollisionShape clone() {
        return new BoxShape(this);
    }

}
