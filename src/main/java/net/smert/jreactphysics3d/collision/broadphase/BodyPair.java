package net.smert.jreactphysics3d.collision.broadphase;

import net.smert.jreactphysics3d.body.BodyIndexPair;
import net.smert.jreactphysics3d.body.CollisionBody;

/**
 * This structure represents a pair of bodies during the broad-phase collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BodyPair {

    /// Pointer to the first body
    public CollisionBody body1;

    /// Pointer to the second body
    public CollisionBody body2;

    /// Return the pair of bodies index
    public BodyIndexPair getBodiesIndexPair() {

        // TODO: Cache object?
        // Construct the pair of body index
        BodyIndexPair indexPair = body1.getID() < body2.getID()
                ? new BodyIndexPair(body1.getID(), body2.getID())
                : new BodyIndexPair(body2.getID(), body1.getID());

        assert (indexPair.first != indexPair.second);
        return indexPair;
    }

}
