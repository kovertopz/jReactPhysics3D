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
package net.smert.jreactphysics3d.collision.broadphase;

import java.util.Objects;
import net.smert.jreactphysics3d.collision.BodyIndexPair;
import net.smert.jreactphysics3d.body.CollisionBody;

/**
 * This structure represents a pair of bodies during the broad-phase collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BodyPair {

    // Pointer to the first body
    private CollisionBody body1;

    // Pointer to the second body
    private CollisionBody body2;

    public CollisionBody getBody1() {
        return body1;
    }

    public CollisionBody getBody2() {
        return body2;
    }

    // Return the pair of bodies index
    public BodyIndexPair newBodiesIndexPair() {

        // TODO: Cache object?
        // Construct the pair of body index
        BodyIndexPair indexPair = body1.getBodyID() < body2.getBodyID()
                ? new BodyIndexPair(body1.getBodyID(), body2.getBodyID())
                : new BodyIndexPair(body2.getBodyID(), body1.getBodyID());
        assert (indexPair.getFirst() != indexPair.getSecond());
        return indexPair;
    }

    public void setBody1(CollisionBody body1) {
        this.body1 = body1;
    }

    public void setBody2(CollisionBody body2) {
        this.body2 = body2;
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 13 * hash + Objects.hashCode(this.body1);
        hash = 13 * hash + Objects.hashCode(this.body2);
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final BodyPair other = (BodyPair) obj;
        if (!Objects.equals(this.body1, other.body1)) {
            return false;
        }
        return Objects.equals(this.body2, other.body2);
    }

}
