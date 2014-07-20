package net.smert.jreactphysics3d.collision.broadphase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.collision.CollisionDetection;
import net.smert.jreactphysics3d.collision.shapes.AABB;

/**
 * This class implements the Sweep-And-Prune (SAP) broad-phase collision detection algorithm. This class implements an
 * array-based implementation of the algorithm from Pierre Terdiman that is described here :
 * www.codercorner.com/SAP.pdf.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class SweepAndPruneAlgorithm extends BroadPhaseAlgorithm {

    /// Invalid array index
    protected static final int INVALID_INDEX = Integer.MAX_VALUE;

    /// Number of sentinel end-points in the array of a given axis
    protected static final int NB_SENTINELS = 2;

    /// Array that contains all the AABB boxes of the broad-phase
    protected BoxAABB[] mBoxes;

    /// Array of end-points on the three axis
    protected EndPoint[][] mEndPoints = {null, null, null};

    /// Number of AABB boxes in the broad-phase
    protected int mNbBoxes;

    /// Max number of boxes in the boxes array
    protected int mNbMaxBoxes;

    /// Indices that are not used by any boxes
    protected ArrayList<Integer> mFreeBoxIndices;

    /// Map a body pointer to a box index
    protected Map<CollisionBody, Integer> mMapBodyToBoxIndex;

    /// Private copy-ructor
    protected SweepAndPruneAlgorithm(SweepAndPruneAlgorithm algorithm) {
    }

    /// Private assignment operator
    protected SweepAndPruneAlgorithm operatorEqual(SweepAndPruneAlgorithm algorithm) {
        return this;
    }

    // Resize the boxes and end-points arrays when it is full
    protected void resizeArrays() {

        // New number of boxes in the array
        int newNbMaxBoxes = mNbMaxBoxes > 0 ? 2 * mNbMaxBoxes : 100;
        int nbEndPoints = mNbBoxes * 2 + NB_SENTINELS;
        int newNbEndPoints = newNbMaxBoxes * 2 + NB_SENTINELS;

        // Allocate memory for the new boxes and end-points arrays
        BoxAABB[] newBoxesArray = new BoxAABB[newNbMaxBoxes];
        EndPoint[] newEndPointsXArray = new EndPoint[newNbEndPoints];
        EndPoint[] newEndPointsYArray = new EndPoint[newNbEndPoints];
        EndPoint[] newEndPointsZArray = new EndPoint[newNbEndPoints];

        assert (newBoxesArray != null);
        assert (newEndPointsXArray != null);
        assert (newEndPointsYArray != null);
        assert (newEndPointsZArray != null);

        // If the arrays were not empty before
        if (mNbBoxes > 0) {

            // Copy the data from the old arrays into the new one
            //memcpy(newBoxesArray, mBoxes, sizeof(BoxAABB) * mNbBoxes);
            //size_t nbBytesNewEndPoints = sizeof(EndPoint) * nbEndPoints;
            //memcpy(newEndPointsXArray, mEndPoints[0], nbBytesNewEndPoints);
            //memcpy(newEndPointsYArray, mEndPoints[1], nbBytesNewEndPoints);
            //memcpy(newEndPointsZArray, mEndPoints[2], nbBytesNewEndPoints);
        } else {   // If the arrays were empty

            // Add the limits endpoints (sentinels) into the array
            int min = encodeFloatIntoInteger(Float.MIN_VALUE);
            int max = encodeFloatIntoInteger(Float.MAX_VALUE);
            newEndPointsXArray[0].setValues(INVALID_INDEX, true, min);
            newEndPointsXArray[1].setValues(INVALID_INDEX, false, max);
            newEndPointsYArray[0].setValues(INVALID_INDEX, true, min);
            newEndPointsYArray[1].setValues(INVALID_INDEX, false, max);
            newEndPointsZArray[0].setValues(INVALID_INDEX, true, min);
            newEndPointsZArray[1].setValues(INVALID_INDEX, false, max);
        }

        // Delete the old arrays
        //delete[] mBoxes;
        //delete[] mEndPoints[0];
        //delete[] mEndPoints[1];
        //delete[] mEndPoints[2];
        // Assign the pointer to the new arrays
        mBoxes = newBoxesArray;
        mEndPoints[0] = newEndPointsXArray;
        mEndPoints[1] = newEndPointsYArray;
        mEndPoints[2] = newEndPointsZArray;

        mNbMaxBoxes = newNbMaxBoxes;
    }

    // Shrink the boxes and end-points arrays when too much memory is allocated
    protected void shrinkArrays() {

        // New number of boxes and end-points in the array
        int nextPowerOf2 = PairManager.computeNextPowerOfTwo((mNbBoxes - 1) / 100);
        int newNbMaxBoxes = (mNbBoxes > 100) ? nextPowerOf2 * 100 : 100;
        int nbEndPoints = mNbBoxes * 2 + NB_SENTINELS;
        int newNbEndPoints = newNbMaxBoxes * 2 + NB_SENTINELS;

        assert (newNbMaxBoxes < mNbMaxBoxes);

        // Sort the list of the free boxes indices in ascending order
        mFreeBoxIndices.sort();

        // Reorganize the boxes inside the boxes array so that all the boxes are at the
        // beginning of the array
        Map<CollisionBody, Integer> newMapBodyToBoxIndex = new HashMap<>();
        Map<CollisionBody, Integer> it;
        for (it = mMapBodyToBoxIndex.begin(); it != mMapBodyToBoxIndex.end(); ++it) {

            CollisionBody body = it.first;
            int boxIndex = it.second;

            // If the box index is outside the range of the current number of boxes
            if (boxIndex >= mNbBoxes) {

                assert (!mFreeBoxIndices.empty());

                // Get a new box index for that body (from the list of free box indices)
                int newBoxIndex = mFreeBoxIndices.front();
                mFreeBoxIndices.pop_front();
                assert (newBoxIndex < mNbBoxes);

                // Copy the box to its new location in the boxes array
                BoxAABB oldBox = mBoxes[boxIndex];
                BoxAABB newBox = mBoxes[newBoxIndex];
                assert (oldBox.body.getID() == body.getID());
                newBox.body = oldBox.body;
                for (int axis = 0; axis < 3; axis++) {

                    // Copy the minimum and maximum end-points indices
                    newBox.min[axis] = oldBox.min[axis];
                    newBox.max[axis] = oldBox.max[axis];

                    // Update the box index of the end-points
                    EndPoint minimumEndPoint = mEndPoints[axis][newBox.min[axis]];
                    EndPoint maximumEndPoint = mEndPoints[axis][newBox.max[axis]];
                    assert (minimumEndPoint.boxID == boxIndex);
                    assert (maximumEndPoint.boxID == boxIndex);
                    minimumEndPoint.boxID = newBoxIndex;
                    maximumEndPoint.boxID = newBoxIndex;
                }

                newMapBodyToBoxIndex.put(body, newBoxIndex);
            } else {
                newMapBodyToBoxIndex.put(body, boxIndex);
            }
        }

        assert (newMapBodyToBoxIndex.size() == mMapBodyToBoxIndex.size());
        mMapBodyToBoxIndex = newMapBodyToBoxIndex;

        // Allocate memory for the new boxes and end-points arrays
        BoxAABB[] newBoxesArray = new BoxAABB[newNbMaxBoxes];
        EndPoint[] newEndPointsXArray = new EndPoint[newNbEndPoints];
        EndPoint[] newEndPointsYArray = new EndPoint[newNbEndPoints];
        EndPoint[] newEndPointsZArray = new EndPoint[newNbEndPoints];

        assert (newBoxesArray != null);
        assert (newEndPointsXArray != null);
        assert (newEndPointsYArray != null);
        assert (newEndPointsZArray != null);

        // Copy the data from the old arrays into the new one
        //memcpy(newBoxesArray, mBoxes, sizeof(BoxAABB) * mNbBoxes);
        //size_t nbBytesNewEndPoints = sizeof(EndPoint) * nbEndPoints;
        //memcpy(newEndPointsXArray, mEndPoints[0], nbBytesNewEndPoints);
        //memcpy(newEndPointsYArray, mEndPoints[1], nbBytesNewEndPoints);
        //memcpy(newEndPointsZArray, mEndPoints[2], nbBytesNewEndPoints);
        // Delete the old arrays
        //delete[] mBoxes;
        //delete[] mEndPoints[0];
        //delete[] mEndPoints[1];
        //delete[] mEndPoints[2];
        // Assign the pointer to the new arrays
        mBoxes = newBoxesArray;
        mEndPoints[0] = newEndPointsXArray;
        mEndPoints[1] = newEndPointsYArray;
        mEndPoints[2] = newEndPointsZArray;

        mNbMaxBoxes = newNbMaxBoxes;
    }

    /// Add an overlapping pair of AABBS
    protected void addPair(CollisionBody body1, CollisionBody body2) {
    }

    // Check for 1D box intersection between two boxes that are sorted on the given axis.
    /// Therefore, only one test is necessary here. We know that the
    /// minimum of box1 cannot be larger that the maximum of box2 on the axis.
    protected boolean testIntersect1DSortedAABBs(BoxAABB box1, AABBInt box2, EndPoint[] endPointsArray, int axis) {
        return !(endPointsArray[box1.max[axis]].value < box2.min[axis]);
    }

    // Check for 2D box intersection. This method is used when we know
    /// that two boxes already overlap on one axis and when want to test
    /// if they also overlap on the two others axis.
    protected boolean testIntersect2D(BoxAABB box1, BoxAABB box2, int axis1, int axis2) {
        return !(box2.max[axis1] < box1.min[axis1] || box1.max[axis1] < box2.min[axis1]
                || box2.max[axis2] < box1.min[axis2] || box1.max[axis2] < box2.min[axis2]);
    }

    // Notify the broad-phase that the AABB of an object has changed.
    /// The input is an AABB with integer coordinates
    protected void updateObjectIntegerAABB(CollisionBody body, AABBInt aabbInt) {

        assert (aabbInt.min[0] > encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (aabbInt.min[1] > encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (aabbInt.min[2] > encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (aabbInt.max[0] < encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (aabbInt.max[1] < encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (aabbInt.max[2] < encodeFloatIntoInteger(Float.MAX_VALUE));

        // Get the corresponding box
        int boxIndex = mMapBodyToBoxIndex.find(body).second;
        BoxAABB box = mBoxes[boxIndex];

        // Current axis
        for (int axis = 0; axis < 3; axis++) {

            // Get the two others axis
            int otherAxis1 = (1 << axis) & 3;
            int otherAxis2 = (1 << otherAxis1) & 3;

            // Get the starting end-point of the current axis
            EndPoint[] startEndPointsCurrentAxis = mEndPoints[axis];

            // -------- Update the minimum end-point ------------//
            EndPoint currentMinEndPoint = startEndPointsCurrentAxis[box.min[axis]];
            assert (currentMinEndPoint.isMin);

            // Get the minimum value of the AABB on the current axis
            int limit = aabbInt.min[axis];

            // If the minimum value of the AABB is smaller
            // than the current minimum endpoint
            if (limit < currentMinEndPoint.value) {

                currentMinEndPoint.value = limit;

                // The minimum end-point is moving left
                EndPoint savedEndPoint = currentMinEndPoint;
                int indexEndPoint = (size_t(currentMinEndPoint)
                        - size_t(startEndPointsCurrentAxis)) / sizeof(EndPoint);
                int savedEndPointIndex = indexEndPoint;

                while ((--currentMinEndPoint).value > limit) {
                    BoxAABB id1 = mBoxes[currentMinEndPoint.boxID];
                    boolean isMin = currentMinEndPoint.isMin;

                    // If it's a maximum end-point
                    if (!isMin) {
                        // The minimum end-point is moving to the left and
                        // passed a maximum end-point. Thus, the boxes start
                        // overlapping on the current axis. Therefore we test
                        // for box intersection
                        if (box != id1) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)
                                    && testIntersect1DSortedAABBs(id1, aabbInt,
                                            startEndPointsCurrentAxis, axis)) {

                                // Add an overlapping pair to the pair manager
                                mPairManager.addPair(body, id1.body);
                            }
                        }

                        id1.max[axis] = indexEndPoint--;
                    } else {
                        id1.min[axis] = indexEndPoint--;
                    }

                    (currentMinEndPoint + 1) = currentMinEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        mBoxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit > currentMinEndPoint.value) {// The minimum of the box has moved to the right

                currentMinEndPoint.value = limit;

                // The minimum en-point is moving right
                EndPoint savedEndPoint = currentMinEndPoint;
                int indexEndPoint = (size_t(currentMinEndPoint)
                        - size_t(startEndPointsCurrentAxis)) / sizeof(EndPoint);
                int savedEndPointIndex = indexEndPoint;

                // For each end-point between the current position of the minimum
                // end-point and the new position of the minimum end-point
                while ((++currentMinEndPoint).value < limit) {
                    BoxAABB id1 = mBoxes[currentMinEndPoint.boxID];
                    boolean isMin = currentMinEndPoint.isMin;

                    // If it's a maximum end-point
                    if (!isMin) {
                        // The minimum end-point is moving to the right and
                        // passed a maximum end-point. Thus, the boxes stop
                        // overlapping on the current axis.
                        if (box != id1) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {

                                // Remove the pair from the pair manager
                                mPairManager.removePair(body.getID(), id1.body.getID());
                            }
                        }

                        id1.max[axis] = indexEndPoint++;
                    } else {
                        id1.min[axis] = indexEndPoint++;
                    }

                    (currentMinEndPoint - 1) = currentMinEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        mBoxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            }

            // ------- Update the maximum end-point ------------ //
            EndPoint currentMaxEndPoint = startEndPointsCurrentAxis[box.max[axis]];
            assert (!currentMaxEndPoint.isMin);

            // Get the maximum value of the AABB on the current axis
            limit = aabbInt.max[axis];

            // If the new maximum value of the AABB is larger
            // than the current maximum end-point value. It means
            // that the AABB is moving to the right.
            if (limit > currentMaxEndPoint.value) {

                currentMaxEndPoint.value = limit;

                EndPoint savedEndPoint = currentMaxEndPoint;
                int indexEndPoint = (size_t(currentMaxEndPoint)
                        - size_t(startEndPointsCurrentAxis)) / sizeof(EndPoint);
                int savedEndPointIndex = indexEndPoint;

                while ((++currentMaxEndPoint).value < limit) {

                    // Get the next end-point
                    BoxAABB id1 = mBoxes[currentMaxEndPoint.boxID];
                    boolean isMin = currentMaxEndPoint.isMin;

                    // If it's a maximum end-point
                    if (isMin) {
                        // The maximum end-point is moving to the right and
                        // passed a minimum end-point. Thus, the boxes start
                        // overlapping on the current axis. Therefore we test
                        // for box intersection
                        if (box != id1) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)
                                    && testIntersect1DSortedAABBs(id1, aabbInt,
                                            startEndPointsCurrentAxis, axis)) {

                                // Add an overlapping pair to the pair manager
                                mPairManager.addPair(body, id1.body);
                            }
                        }

                        id1.min[axis] = indexEndPoint++;
                    } else {
                        id1.max[axis] = indexEndPoint++;
                    }

                    (currentMaxEndPoint - 1) = currentMaxEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        mBoxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit < currentMaxEndPoint.value) {   // If the AABB is moving to the left 
                currentMaxEndPoint.value = limit;

                EndPoint savedEndPoint = currentMaxEndPoint;
                int indexEndPoint = (size_t(currentMaxEndPoint)
                        - size_t(startEndPointsCurrentAxis)) / sizeof(EndPoint);
                int savedEndPointIndex = indexEndPoint;

                // For each end-point between the current position of the maximum
                // end-point and the new position of the maximum end-point
                while ((--currentMaxEndPoint).value > limit) {
                    BoxAABB id1 = mBoxes[currentMaxEndPoint.boxID];
                    boolean isMin = currentMaxEndPoint.isMin;

                    // If it's a minimum end-point
                    if (isMin) {
                        // The maximum end-point is moving to the right and
                        // passed a minimum end-point. Thus, the boxes stop
                        // overlapping on the current axis.
                        if (box != id1) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {

                                // Remove the pair from the pair manager
                                mPairManager.removePair(body.getID(), id1.body.getID());
                            }
                        }

                        id1.min[axis] = indexEndPoint--;
                    } else {
                        id1.max[axis] = indexEndPoint--;
                    }

                    (currentMaxEndPoint + 1) = currentMaxEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        mBoxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            }
        }
    }

    // Constructor
    public SweepAndPruneAlgorithm(CollisionDetection collisionDetection) {
        super(collisionDetection);

        mBoxes = null;
        mNbBoxes = 0;
        mNbMaxBoxes = 0;
    }

    /// Encode a floating value into a integer value in order to
    /// work with integer comparison in the Sweep-And-Prune algorithm
    /// because it is faster. The main issue when encoding floating
    /// number into integer is to keep to sorting order. This is a
    /// problem for negative float number. This article describes
    /// how to solve this issue : http://www.stereopsis.com/radix.html
    public int encodeFloatIntoInteger(float number) {
        int intNumber = Float.floatToIntBits(number) & 0xFFFFFFFF;

        // If it's a negative number
        if ((intNumber & 0x80000000) == 0x80000000) {
            intNumber = ~intNumber;
        } else {     // If it is a positive number
            intNumber |= 0x80000000l;
        }

        return intNumber;
    }

    // Notify the broad-phase that the AABB of an object has changed
    @Override
    public void updateObject(CollisionBody body, AABB aabb) {

        // Compute the corresponding AABB with integer coordinates
        AABBInt aabbInt = new AABBInt(aabb);

        // Call the update object method that uses an AABB with integer coordinates
        updateObjectIntegerAABB(body, aabbInt);
    }

    // Notify the broad-phase about a new object in the world
    /// This method adds the AABB of the object ion to broad-phase
    @Override
    public void addObject(CollisionBody body, AABB aabb) {

        int boxIndex;

        assert (encodeFloatIntoInteger(aabb.getMin().x) > encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (encodeFloatIntoInteger(aabb.getMin().y) > encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (encodeFloatIntoInteger(aabb.getMin().z) > encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (encodeFloatIntoInteger(aabb.getMax().x) < encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (encodeFloatIntoInteger(aabb.getMax().y) < encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (encodeFloatIntoInteger(aabb.getMax().z) < encodeFloatIntoInteger(Float.MAX_VALUE));

        // If the index of the first free box is valid (means that
        // there is a bucket in the middle of the array that doesn't
        // contain a box anymore because it has been removed)
        if (!mFreeBoxIndices.empty()) {
            boxIndex = mFreeBoxIndices.back();
            mFreeBoxIndices.pop_back();
        } else {
            // If the array boxes and end-points arrays are full
            if (mNbBoxes == mNbMaxBoxes) {
                // Resize the arrays to make them larger
                resizeArrays();
            }

            boxIndex = mNbBoxes;
        }

        // Move the maximum limit end-point two elements further
        // at the end-points array in all three axis
        int indexLimitEndPoint = 2 * mNbBoxes + NB_SENTINELS - 1;
        for (int axis = 0; axis < 3; axis++) {
            EndPoint maxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint];
            assert (mEndPoints[axis][0].boxID == INVALID_INDEX && mEndPoints[axis][0].isMin);
            assert (maxLimitEndPoint.boxID == INVALID_INDEX && !maxLimitEndPoint.isMin);
            EndPoint newMaxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint + 2];
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.boxID, maxLimitEndPoint.isMin,
                    maxLimitEndPoint.value);
        }

        // Create a new box
        BoxAABB box = mBoxes[boxIndex];
        box.body = body;
        int maxEndPointValue = encodeFloatIntoInteger(Float.MAX_VALUE) - 1;
        int minEndPointValue = encodeFloatIntoInteger(Float.MAX_VALUE) - 2;
        for (int axis = 0; axis < 3; axis++) {
            box.min[axis] = indexLimitEndPoint;
            box.max[axis] = indexLimitEndPoint + 1;
            EndPoint minimumEndPoint = mEndPoints[axis][box.min[axis]];
            minimumEndPoint.setValues(boxIndex, true, minEndPointValue);
            EndPoint maximumEndPoint = mEndPoints[axis][box.max[axis]];
            maximumEndPoint.setValues(boxIndex, false, maxEndPointValue);
        }

        // Add the body pointer to box index mapping
        mMapBodyToBoxIndex.put(body, boxIndex);

        mNbBoxes++;

        // Call the update method to put the end-points of the new AABB at the
        // correct position in the array. This will also create the overlapping
        // pairs in the pair manager if the new AABB is overlapping with others
        // AABBs
        updateObject(body, aabb);
    }

    // Notify the broad-phase about an object that has been removed from the world
    @Override
    public void removeObject(CollisionBody body) {

        assert (mNbBoxes > 0);

        // Call the update method with an AABB that is very far away
        // in order to remove all overlapping pairs from the pair manager
        int maxEndPointValue = encodeFloatIntoInteger(Float.MAX_VALUE) - 1;
        int minEndPointValue = encodeFloatIntoInteger(Float.MAX_VALUE) - 2;
        AABBInt aabbInt = new AABBInt(minEndPointValue, maxEndPointValue);
        updateObjectIntegerAABB(body, aabbInt);

        // Get the corresponding box
        int boxIndex = mMapBodyToBoxIndex.find(body).second;

        // Remove the end-points of the box by moving the maximum end-points two elements back in
        // the end-points array
        int indexLimitEndPoint = 2 * mNbBoxes + NB_SENTINELS - 1;
        for (int axis = 0; axis < 3; axis++) {
            EndPoint maxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint];
            assert (mEndPoints[axis][0].boxID == INVALID_INDEX && mEndPoints[axis][0].isMin);
            assert (maxLimitEndPoint.boxID == INVALID_INDEX && !maxLimitEndPoint.isMin);
            EndPoint newMaxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint - 2];
            assert (mEndPoints[axis][indexLimitEndPoint - 1].boxID == boxIndex);
            assert (!mEndPoints[axis][indexLimitEndPoint - 1].isMin);
            assert (newMaxLimitEndPoint.boxID == boxIndex);
            assert (newMaxLimitEndPoint.isMin);
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.boxID, maxLimitEndPoint.isMin,
                    maxLimitEndPoint.value);
        }

        // Add the box index into the list of free indices
        mFreeBoxIndices.push_back(boxIndex);

        mMapBodyToBoxIndex.erase(body);
        mNbBoxes--;

        // Check if we need to shrink the allocated memory
        int nextPowerOf2 = PairManager.computeNextPowerOfTwo((mNbBoxes - 1) / 100);
        if (nextPowerOf2 * 100 < mNbMaxBoxes) {
            shrinkArrays();
        }
    }

}
