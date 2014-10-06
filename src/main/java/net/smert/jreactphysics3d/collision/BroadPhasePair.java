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
package net.smert.jreactphysics3d.collision;

import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This structure represents a pair of bodies during the broad-phase collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BroadPhasePair {

    // Pointer to the first body
    private final CollisionBody body1;

    // Pointer to the second body
    private final CollisionBody body2;

    // Previous cached separating axis
    private final Vector3 previousSeparatingAxis;

    // Constructor
    public BroadPhasePair(CollisionBody body1, CollisionBody body2) {
        this.body1 = body1;
        this.body2 = body2;
        previousSeparatingAxis = new Vector3(1.0f, 1.0f, 1.0f);
    }

    public CollisionBody getBody1() {
        return body1;
    }

    public CollisionBody getBody2() {
        return body2;
    }

    public Vector3 getPreviousSeparatingAxis() {
        return previousSeparatingAxis;
    }

    // Return the pair of bodies index
    public BodyIndexPair newBodiesIndexPair() {
        return ComputeBodiesIndexPair(body1, body2);
    }

    // Return the pair of bodies index
    public static BodyIndexPair ComputeBodiesIndexPair(CollisionBody body1, CollisionBody body2) {

        // Construct the pair of body index
        BodyIndexPair indexPair = body1.getBodyID() < body2.getBodyID()
                ? new BodyIndexPair(body1.getBodyID(), body2.getBodyID())
                : new BodyIndexPair(body2.getBodyID(), body1.getBodyID());
        assert (indexPair.getFirst() != indexPair.getSecond());
        return indexPair;
    }

}
