package net.smert.jreactphysics3d.collision.broadphase;

import java.util.ArrayList;
import java.util.Collections;
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

    /// Add an overlapping pair of AABBS
    protected void addPair(CollisionBody body1, CollisionBody body2) {
        // TODO: remove unused method
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
            System.arraycopy(mBoxes, 0, newBoxesArray, 0, mNbBoxes);
            System.arraycopy(mEndPoints[0], 0, newEndPointsXArray, 0, nbEndPoints);
            System.arraycopy(mEndPoints[1], 0, newEndPointsYArray, 0, nbEndPoints);
            System.arraycopy(mEndPoints[2], 0, newEndPointsZArray, 0, nbEndPoints);
        } else {   // If the arrays were empty

            // Add the limits endpoints (sentinels) into the array
            int min = Utils.encodeFloatIntoInteger(Float.MIN_VALUE);
            int max = Utils.encodeFloatIntoInteger(Float.MAX_VALUE);
            newEndPointsXArray[0] = new EndPoint();
            newEndPointsXArray[0].setValues(INVALID_INDEX, true, min);
            newEndPointsXArray[1] = new EndPoint();
            newEndPointsXArray[1].setValues(INVALID_INDEX, false, max);
            newEndPointsYArray[0] = new EndPoint();
            newEndPointsYArray[0].setValues(INVALID_INDEX, true, min);
            newEndPointsYArray[1] = new EndPoint();
            newEndPointsYArray[1].setValues(INVALID_INDEX, false, max);
            newEndPointsZArray[0] = new EndPoint();
            newEndPointsZArray[0].setValues(INVALID_INDEX, true, min);
            newEndPointsZArray[1] = new EndPoint();
            newEndPointsZArray[1].setValues(INVALID_INDEX, false, max);
        }

        // Delete the old arrays
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
        int nextPowerOf2 = PairManager.ComputeNextPowerOfTwo((mNbBoxes - 1) / 100);
        int newNbMaxBoxes = (mNbBoxes > 100) ? nextPowerOf2 * 100 : 100;
        int nbEndPoints = mNbBoxes * 2 + NB_SENTINELS;
        int newNbEndPoints = newNbMaxBoxes * 2 + NB_SENTINELS;

        assert (newNbMaxBoxes < mNbMaxBoxes);

        // Sort the list of the free boxes indices in ascending order
        Collections.sort(mFreeBoxIndices);

        // Reorganize the boxes inside the boxes array so that all the boxes are at the
        // beginning of the array
        Map<CollisionBody, Integer> newMapBodyToBoxIndex = new HashMap<>();
        for (Map.Entry it : mMapBodyToBoxIndex.entrySet()) {

            CollisionBody body = (CollisionBody) it.getKey();
            int boxIndex = (int) it.getValue();

            // If the box index is outside the range of the current number of boxes
            if (boxIndex >= mNbBoxes) {

                assert (!mFreeBoxIndices.isEmpty());

                // Get a new box index for that body (from the list of free box indices)
                int newBoxIndex = mFreeBoxIndices.get(0);
                mFreeBoxIndices.remove(0);
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
        System.arraycopy(mBoxes, 0, newBoxesArray, 0, mNbBoxes);
        System.arraycopy(mEndPoints[0], 0, newEndPointsXArray, 0, nbEndPoints);
        System.arraycopy(mEndPoints[1], 0, newEndPointsYArray, 0, nbEndPoints);
        System.arraycopy(mEndPoints[2], 0, newEndPointsZArray, 0, nbEndPoints);

        // Delete the old arrays
        // Assign the pointer to the new arrays
        mBoxes = newBoxesArray;
        mEndPoints[0] = newEndPointsXArray;
        mEndPoints[1] = newEndPointsYArray;
        mEndPoints[2] = newEndPointsZArray;

        mNbMaxBoxes = newNbMaxBoxes;
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

        assert (aabbInt.min[0] > Utils.encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (aabbInt.min[1] > Utils.encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (aabbInt.min[2] > Utils.encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (aabbInt.max[0] < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (aabbInt.max[1] < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (aabbInt.max[2] < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));

        // Get the corresponding box
        int boxIndex = mMapBodyToBoxIndex.get(body);
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
            int currentMinEndPointIndex = box.min[axis];
            assert (currentMinEndPoint.isMin);

            // Get the minimum value of the AABB on the current axis
            int limit = aabbInt.min[axis];

            // If the minimum value of the AABB is smaller
            // than the current minimum endpoint
            if (limit < currentMinEndPoint.value) {

                currentMinEndPoint.value = limit;

                // The minimum end-point is moving left
                EndPoint savedEndPoint = currentMinEndPoint;
                int indexEndPoint = currentMinEndPointIndex;
                int savedEndPointIndex = indexEndPoint;

                while (startEndPointsCurrentAxis[--currentMinEndPointIndex].value > limit) {
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

                    startEndPointsCurrentAxis[currentMinEndPointIndex + 1] = currentMinEndPoint;
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
                int indexEndPoint = currentMinEndPointIndex;
                int savedEndPointIndex = indexEndPoint;

                // For each end-point between the current position of the minimum
                // end-point and the new position of the minimum end-point
                while (startEndPointsCurrentAxis[++currentMinEndPointIndex].value < limit) {

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

                    startEndPointsCurrentAxis[currentMinEndPointIndex + 1] = currentMinEndPoint;
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

            int currentMaxEndPointIndex = box.max[axis];
            assert (!currentMaxEndPoint.isMin);

            // Get the maximum value of the AABB on the current axis
            limit = aabbInt.max[axis];

            // If the new maximum value of the AABB is larger
            // than the current maximum end-point value. It means
            // that the AABB is moving to the right.
            if (limit > currentMaxEndPoint.value) {

                currentMaxEndPoint.value = limit;

                EndPoint savedEndPoint = currentMaxEndPoint;
                int indexEndPoint = currentMinEndPointIndex;
                int savedEndPointIndex = indexEndPoint;

                while (startEndPointsCurrentAxis[++currentMaxEndPointIndex].value < limit) {

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

                    startEndPointsCurrentAxis[currentMaxEndPointIndex + 1] = currentMaxEndPoint;
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
                int indexEndPoint = currentMaxEndPointIndex;
                int savedEndPointIndex = indexEndPoint;

                // For each end-point between the current position of the maximum
                // end-point and the new position of the maximum end-point
                while (startEndPointsCurrentAxis[--currentMaxEndPointIndex].value > limit) {
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

                    startEndPointsCurrentAxis[currentMaxEndPointIndex + 1] = currentMaxEndPoint;
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

        mFreeBoxIndices = new ArrayList<>();
        mMapBodyToBoxIndex = new HashMap<>();
    }

    // Notify the broad-phase about a new object in the world
    /// This method adds the AABB of the object ion to broad-phase
    @Override
    public void addObject(CollisionBody body, AABB aabb) {

        int boxIndex;

        assert (Utils.encodeFloatIntoInteger(aabb.getMin().x) > Utils.encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMin().y) > Utils.encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMin().z) > Utils.encodeFloatIntoInteger(Float.MIN_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMax().x) < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMax().y) < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMax().z) < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));

        // If the index of the first free box is valid (means that
        // there is a bucket in the middle of the array that doesn't
        // contain a box anymore because it has been removed)
        if (!mFreeBoxIndices.isEmpty()) {
            int lastIndex = mFreeBoxIndices.size() - 1;
            boxIndex = mFreeBoxIndices.get(lastIndex);
            mFreeBoxIndices.remove(lastIndex);
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
            if (mEndPoints[axis][indexLimitEndPoint + 2] == null) {
                mEndPoints[axis][indexLimitEndPoint + 2] = new EndPoint();
            }
            EndPoint newMaxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint + 2];
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.boxID, maxLimitEndPoint.isMin,
                    maxLimitEndPoint.value);
        }

        // Create a new box
        if (mBoxes[boxIndex] == null) {
            mBoxes[boxIndex] = new BoxAABB();
        }
        BoxAABB box = mBoxes[boxIndex];
        box.body = body;
        int maxEndPointValue = Utils.encodeFloatIntoInteger(Float.MAX_VALUE) - 1;
        int minEndPointValue = Utils.encodeFloatIntoInteger(Float.MAX_VALUE) - 2;
        for (int axis = 0; axis < 3; axis++) {
            box.min[axis] = indexLimitEndPoint;
            box.max[axis] = indexLimitEndPoint + 1;
            EndPoint minimumEndPoint = mEndPoints[axis][box.min[axis]];
            minimumEndPoint.setValues(boxIndex, true, minEndPointValue);
            if (mEndPoints[axis][box.max[axis]] == null) {
                mEndPoints[axis][box.max[axis]] = new EndPoint();
            }
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
        int maxEndPointValue = Utils.encodeFloatIntoInteger(Float.MAX_VALUE) - 1;
        int minEndPointValue = Utils.encodeFloatIntoInteger(Float.MAX_VALUE) - 2;
        AABBInt aabbInt = new AABBInt(minEndPointValue, maxEndPointValue);
        updateObjectIntegerAABB(body, aabbInt);

        // Get the corresponding box
        int boxIndex = mMapBodyToBoxIndex.get(body);

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
        mFreeBoxIndices.add(boxIndex);

        mMapBodyToBoxIndex.remove(body);
        mNbBoxes--;

        // Check if we need to shrink the allocated memory
        int nextPowerOf2 = PairManager.ComputeNextPowerOfTwo((mNbBoxes - 1) / 100);
        if (nextPowerOf2 * 100 < mNbMaxBoxes) {
            shrinkArrays();
        }
    }

    // Notify the broad-phase that the AABB of an object has changed
    @Override
    public void updateObject(CollisionBody body, AABB aabb) {

        // Compute the corresponding AABB with integer coordinates
        AABBInt aabbInt = new AABBInt(aabb);

        // Call the update object method that uses an AABB with integer coordinates
        updateObjectIntegerAABB(body, aabbInt);
    }

}
