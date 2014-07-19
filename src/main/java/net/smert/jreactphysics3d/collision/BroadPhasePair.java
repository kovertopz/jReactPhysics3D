package net.smert.jreactphysics3d.collision;

import net.smert.jreactphysics3d.body.BodyIndexPair;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This structure represents a pair of bodies during the broad-phase collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BroadPhasePair {

    /// Pointer to the first body
    public CollisionBody body1;

    /// Pointer to the second body
    public CollisionBody body2;

    /// Previous cached separating axis
    public Vector3 previousSeparatingAxis;

    // Constructor
    public BroadPhasePair(CollisionBody body1, CollisionBody body2) {
        this.body1 = body1;
        this.body2 = body2;
        previousSeparatingAxis = new Vector3(1.0f, 1.0f, 1.0f);
    }

    // Return the pair of bodies index
    public BodyIndexPair computeBodiesIndexPair(CollisionBody body1, CollisionBody body2) {

        // Construct the pair of body index
        BodyIndexPair indexPair = body1.getID() < body2.getID()
                ? new BodyIndexPair(body1.getID(), body2.getID())
                : new BodyIndexPair(body2.getID(), body1.getID());
        assert (indexPair.first != indexPair.second);
        return indexPair;
    }

    // Return the pair of bodies index
    public BodyIndexPair getBodiesIndexPair() {

        return computeBodiesIndexPair(body1, body2);
    }

    // Smaller than operator
    public boolean operatorLessThan(BroadPhasePair broadPhasePair2) {
        return (body1 < broadPhasePair2.body1 ? true : (body2 < broadPhasePair2.body2));
    }

    // Larger than operator
    public boolean operatorGreaterThan(BroadPhasePair broadPhasePair2) {
        return (body1 > broadPhasePair2.body1 ? true : (body2 > broadPhasePair2.body2));
    }

    // Equal operator
    public boolean operatorEquals(BroadPhasePair broadPhasePair2) {
        return (body1 == broadPhasePair2.body1 && body2 == broadPhasePair2.body2);
    }

    // Not equal operator
    public boolean operatorNotEquals(BroadPhasePair broadPhasePair2) {
        return (body1 != broadPhasePair2.body1 || body2 != broadPhasePair2.body2);
    }

}
