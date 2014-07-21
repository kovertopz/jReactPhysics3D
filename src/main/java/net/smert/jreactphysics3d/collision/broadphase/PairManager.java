package net.smert.jreactphysics3d.collision.broadphase;

import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.collision.CollisionDetection;

/**
 * This class is a data-structure contains the pairs of bodies that are overlapping during the broad-phase collision
 * detection. This class implements the pair manager described by Pierre Terdiman in www.codercorner.com/SAP.pdf.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class PairManager {

    /// Number of elements in the hash table
    private int mNbElementsHashTable;

    /// Hash mask for the hash function
    private int mHashMask;

    /// Number of overlapping pairs
    private int mNbOverlappingPairs;

    /// Hash table that contains the offset of the first pair of the list of
    /// pairs with the same hash value in the "overlappingPairs" array
    private int[] mHashTable;

    /// Array that contains for each offset, the offset of the next pair with
    /// the same hash value for a given same hash value
    private int[] mOffsetNextPair;

    /// Array that contains the overlapping pairs
    private BodyPair[] mOverlappingPairs;

    /// Invalid ID
    private static final int INVALID_INDEX = Integer.MAX_VALUE;

    /// Reference to the collision detection
    private CollisionDetection mCollisionDetection;

    // This method returns an hash value for a 32 bits key.
    /// using Thomas Wang's hash technique.
    /// This hash function can be found at :
    /// http://www.concentric.net/~ttwang/tech/inthash.htm
    private int computeHash32Bits(int key) {
        key += ~(key << 15);
        key ^= (key >> 10);
        key += (key << 3);
        key ^= (key >> 6);
        key += ~(key << 11);
        key ^= (key >> 16);
        return key;
    }

    // Compute the hash value of two bodies
    private int computeHashBodies(int id1, int id2) {
        return computeHash32Bits(id1 | (id2 << 16));
    }

    // Compute the offset of a given pair in the array of overlapping pairs
    private int computePairOffset(BodyPair pair) {
        for (int i = 0; i < mOverlappingPairs.length; i++) {
            if (pair.equals(mOverlappingPairs[i])) {
                return i;
            }
        }
        return -1;
    }

    // Find a pair given two body IDs and an hash value.
    /// This internal version is used to avoid computing multiple times in the
    /// caller method
    private BodyPair findPairWithHashValue(int id1, int id2, int hashValue) {

        // Check if the hash table has been allocated yet
        if (mHashTable == null) {
            return null;
        }

        // Look for the pair in the set of overlapping pairs
        return lookForAPair(id1, id2, hashValue);
    }

    // Return true if pair1 and pair2 are the same
    private boolean isDifferentPair(BodyPair pair1, int pair2ID1, int pair2ID2) {
        return (pair2ID1 != pair1.body1.getID() || pair2ID2 != pair1.body2.getID());
    }

    // Look for a pair in the set of overlapping pairs
    public BodyPair lookForAPair(int id1, int id2, int hashValue) {

        // Look for the pair in the set of overlapping pairs
        int offset = mHashTable[hashValue];
        while (offset != INVALID_INDEX && isDifferentPair(mOverlappingPairs[offset], id1, id2)) {
            offset = mOffsetNextPair[offset];
        }

        // If the pair has not been found in the overlapping pairs
        if (offset == INVALID_INDEX) {
            return null;
        }

        assert (offset < mNbOverlappingPairs);

        // The pair has been found in the set of overlapping pairs, then
        // we return a pointer to it
        return mOverlappingPairs[offset];
    }

    // Reallocate more pairs
    private void reallocatePairs() {

        // Reallocate the hash table and initialize it
        mHashTable = new int[mNbElementsHashTable];
        assert (mHashTable != null);
        for (int i = 0; i < mNbElementsHashTable; i++) {
            mHashTable[i] = INVALID_INDEX;
        }

        // Reallocate the overlapping pairs
        BodyPair[] newOverlappingPairs = new BodyPair[mNbElementsHashTable];
        int[] newOffsetNextPair = new int[mNbElementsHashTable];

        assert (newOverlappingPairs != null);
        assert (newOffsetNextPair != null);

        // If there is already some overlapping pairs
        if (mNbOverlappingPairs > 0) {
            // Copy the pairs to the new location
            System.arraycopy(mOverlappingPairs, 0, newOverlappingPairs, 0, mNbOverlappingPairs);
        }

        // Recompute the hash table with the new hash values
        for (int i = 0; i < mNbOverlappingPairs; i++) {
            int newHashValue = computeHashBodies(mOverlappingPairs[i].body1.getID(),
                    mOverlappingPairs[i].body2.getID()) & mHashMask;
            newOffsetNextPair[i] = mHashTable[newHashValue];
            mHashTable[newHashValue] = i;
        }

        // Delete the old pairs
        // Replace by the new data
        mOverlappingPairs = newOverlappingPairs;
        mOffsetNextPair = newOffsetNextPair;
    }

    // Internal method to remove a pair from the set of overlapping pair
    private void removePairWithHashValue(int id1, int id2, int hashValue, int indexPair) {

        // Get the initial offset of the pairs with
        // the corresponding hash value
        int offset = mHashTable[hashValue];
        assert (offset != INVALID_INDEX);

        // Look for the pair in the set of overlapping pairs
        int previousPair = INVALID_INDEX;
        while (offset != indexPair) {
            previousPair = offset;
            offset = mOffsetNextPair[offset];
        }

        // If the pair was the first one with this hash
        // value in the hash table
        if (previousPair == INVALID_INDEX) {
            // Replace the pair to remove in the
            // hash table by the next one
            mHashTable[hashValue] = mOffsetNextPair[indexPair];
        } else {    // If the pair was not the first one
            // Replace the pair to remove in the
            // hash table by the next one
            assert (mOffsetNextPair[previousPair] == indexPair);
            mOffsetNextPair[previousPair] = mOffsetNextPair[indexPair];
        }

        int indexLastPair = mNbOverlappingPairs - 1;

        // If the pair to remove is the last one in the list
        if (indexPair == indexLastPair) {

            // We simply decrease the number of overlapping pairs
            mNbOverlappingPairs--;
        } else {    // If the pair to remove is in the middle of the list

            // Now, we want to move the last pair into the location that is
            // now free because of the pair we want to remove
            // Get the last pair
            BodyPair lastPair = mOverlappingPairs[indexLastPair];
            int lastPairHashValue = computeHashBodies(lastPair.body1.getID(),
                    lastPair.body2.getID()) & mHashMask;

            // Compute the initial offset of the last pair
            offset = mHashTable[lastPairHashValue];
            assert (offset != INVALID_INDEX);

            // Go through the pairs with the same hash value
            // and find the offset of the last pair
            int previous = INVALID_INDEX;
            while (offset != indexLastPair) {
                previous = offset;
                offset = mOffsetNextPair[offset];
            }

            // If the last pair is not the first one with this hash value
            if (previous != INVALID_INDEX) {

                // Remove the offset of the last pair in the "nextOffset" array
                assert (mOffsetNextPair[previous] == indexLastPair);
                mOffsetNextPair[previous] = mOffsetNextPair[indexLastPair];
            } else {    // If the last pair is the first offset with this hash value

                // Remove the offset of the last pair in the "nextOffset" array
                mHashTable[lastPairHashValue] = mOffsetNextPair[indexLastPair];
            }

            // Replace the pair to remove by the last pair in
            // the overlapping pairs array
            mOverlappingPairs[indexPair] = mOverlappingPairs[indexLastPair];
            mOffsetNextPair[indexPair] = mHashTable[lastPairHashValue];
            mHashTable[lastPairHashValue] = indexPair;

            mNbOverlappingPairs--;
        }
    }

    // Try to reduce the allocated memory by the pair manager
    private void shrinkMemory() {

        // Check if the allocated memory can be reduced
        int correctNbElementsHashTable = ComputeNextPowerOfTwo(mNbOverlappingPairs);
        if (mNbElementsHashTable == correctNbElementsHashTable) {
            return;
        }

        // Reduce the allocated memory
        mNbElementsHashTable = correctNbElementsHashTable;
        mHashMask = mNbElementsHashTable - 1;
        reallocatePairs();
    }

    // Sort the bodies according to their IDs (smallest ID first)
    private void sortBodiesUsingID(CollisionBody body1, CollisionBody body2) {
        // TODO: remove unused method
    }

    // Sort the IDs (smallest ID first)
    private void sortIDs(int id1, int id2) {
        // TODO: remove unused method
    }

    // Constructor of PairManager
    public PairManager(CollisionDetection collisionDetection) {
        mNbElementsHashTable = 0;
        mHashMask = 0;
        mNbOverlappingPairs = 0;
        mHashTable = null;
        mOffsetNextPair = null;
        mOverlappingPairs = null;
        mCollisionDetection = collisionDetection;
    }

    // Return a pointer to the first overlapping pair (used to iterate over the overlapping pairs) or
    // returns 0 if there is no overlapping pairs.
    public BodyPair beginOverlappingPairsPointer() {
        return mOverlappingPairs[0];
    }

    // Return a pointer to the last overlapping pair (used to iterate over the overlapping pairs) or
    // returns 0 if there is no overlapping pairs.
    public BodyPair endOverlappingPairsPointer() {
        if (mNbOverlappingPairs > 0) {
            return mOverlappingPairs[mNbOverlappingPairs - 1];
        } else {
            return mOverlappingPairs[0];
        }
    }

    // Return the number of overlapping pairs
    public int getNbOverlappingPairs() {
        return mNbOverlappingPairs;
    }

    // Find a pair given two body IDs
    public BodyPair findPair(int id1, int id2) {

        // Check if the hash table has been allocated yet
        if (mHashTable == null) {
            return null;
        }

        // Sort the IDs
        if (id1 > id2) {
            int temp = id2;
            id2 = id1;
            id1 = temp;
        }

        // Compute the hash value of the pair to find
        int hashValue = computeHashBodies(id1, id2) & mHashMask;

        // Look for the pair in the set of overlapping pairs
        return lookForAPair(id1, id2, hashValue);
    }

    // Add a pair of bodies in the pair manager and returns a pointer to that pair.
    /// If the pair to add does not already exist in the set of
    /// overlapping pairs, it will be created and if it already exists, we only
    /// return a pointer to that pair.
    public BodyPair addPair(CollisionBody body1, CollisionBody body2) {

        // Sort the bodies to have the body with smallest ID first
        // If the ID of body1 is larger than the ID of body 2
        if (body1.getID() > body2.getID()) {

            // Swap the two bodies pointers
            CollisionBody temp = body2;
            body2 = body1;
            body1 = temp;
        }

        // Get the bodies IDs
        int id1 = body1.getID();
        int id2 = body2.getID();

        // Compute the hash value of the two bodies
        int hashValue = computeHashBodies(id1, id2) & mHashMask;

        // Try to find the pair in the current overlapping pairs.
        BodyPair pair = findPairWithHashValue(id1, id2, hashValue);

        // If the pair is already in the set of overlapping pairs
        if (pair != null) {
            // We only return a pointer to that pair
            return pair;
        }

        // If we need to allocate more pairs in the set of overlapping pairs
        if (mNbOverlappingPairs >= mNbElementsHashTable) {
            // Increase the size of the hash table (always a power of two)
            mNbElementsHashTable = ComputeNextPowerOfTwo(mNbOverlappingPairs + 1);

            // Compute the new hash mask with the new hash size
            mHashMask = mNbElementsHashTable - 1;

            // Reallocate more pairs
            reallocatePairs();

            // Compute the new hash value (using the new hash size and hash mask)
            hashValue = computeHashBodies(id1, id2) & mHashMask;
        }

        // Create the new overlapping pair
        BodyPair newPair = mOverlappingPairs[mNbOverlappingPairs];
        newPair.body1 = body1;
        newPair.body2 = body2;

        // Put the new pair as the initial pair with this hash value
        mOffsetNextPair[mNbOverlappingPairs] = mHashTable[hashValue];
        mHashTable[hashValue] = mNbOverlappingPairs++;

        // Notify the collision detection about this new overlapping pair
        mCollisionDetection.broadPhaseNotifyAddedOverlappingPair(newPair);

        // Return a pointer to the new created pair
        return newPair;
    }

    // Remove a pair of bodies from the pair manager.
    /// This method returns true if the pair has been found and removed.
    public boolean removePair(int id1, int id2) {

        // Sort the bodies IDs
        if (id1 > id2) {
            int temp = id2;
            id2 = id1;
            id1 = temp;
        }

        // Compute the hash value of the pair to remove
        int hashValue = computeHashBodies(id1, id2) & mHashMask;

        // Find the pair to remove
        BodyPair pair = findPairWithHashValue(id1, id2, hashValue);

        // If we have not found the pair
        if (pair == null) {
            return false;
        }

        assert (pair.body1.getID() == id1);
        assert (pair.body2.getID() == id2);

        // Notify the collision detection about this removed overlapping pair
        mCollisionDetection.broadPhaseNotifyRemovedOverlappingPair(pair);

        // Remove the pair from the set of overlapping pairs
        removePairWithHashValue(id1, id2, hashValue, computePairOffset(pair));

        // Try to shrink the memory used by the pair manager
        shrinkMemory();

        return true;
    }

    // Return the next power of two of a 32bits integer using a SWAR algorithm
    public static int ComputeNextPowerOfTwo(int number) {
        number |= (number >> 1);
        number |= (number >> 2);
        number |= (number >> 4);
        number |= (number >> 8);
        number |= (number >> 16);
        return number + 1;
    }

}
