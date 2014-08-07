package net.smert.jreactphysics3d.collision.broadphase;

import java.util.Objects;
import net.smert.jreactphysics3d.body.BodyIndexPair;
import net.smert.jreactphysics3d.body.CollisionBody;

/**
 * This structure represents a pair of bodies during the broad-phase collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BodyPair {

    // Pointer to the first body
    public CollisionBody body1;

    // Pointer to the second body
    public CollisionBody body2;

    // Return the pair of bodies index
    public BodyIndexPair getBodiesIndexPair() {

        // TODO: Cache object?
        // Construct the pair of body index
        BodyIndexPair indexPair = body1.getID() < body2.getID()
                ? new BodyIndexPair(body1.getID(), body2.getID())
                : new BodyIndexPair(body2.getID(), body1.getID());

        assert (indexPair.first != indexPair.second);
        return indexPair;
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
