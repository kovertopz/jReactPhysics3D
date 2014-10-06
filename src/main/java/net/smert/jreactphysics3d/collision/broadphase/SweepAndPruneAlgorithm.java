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

    // Invalid array index
    protected final static int INVALID_INDEX = Integer.MAX_VALUE;

    // Number of sentinel end-points in the array of a given axis
    protected final static int NUM_SENTINELS = 2;

    // Number of AABB boxes in the broad-phase
    protected int numBoxes;

    // Max number of boxes in the boxes array
    protected int numMaxBoxes;

    // Indices that are not used by any boxes
    protected final ArrayList<Integer> freeBoxIndices;

    // Array that contains all the AABB boxes of the broad-phase
    protected BoxAABB[] boxes;

    // Array of end-points on the three axis
    protected final EndPoint[][] endPoints = {null, null, null};

    // Map a body pointer to a box index
    protected final Map<CollisionBody, Integer> mapBodyToBoxIndex;

    // Constructor
    public SweepAndPruneAlgorithm(CollisionDetection collisionDetection) {
        super(collisionDetection);
        numBoxes = 0;
        numMaxBoxes = 0;
        boxes = null;
        freeBoxIndices = new ArrayList<>();
        mapBodyToBoxIndex = new HashMap<>();
    }

    // Resize the boxes and end-points arrays when it is full
    protected void resizeArrays() {

        // New number of boxes in the array
        int newNumMaxBoxes = numMaxBoxes > 0 ? 2 * numMaxBoxes : 100;
        int numEndPoints = numBoxes * 2 + NUM_SENTINELS;
        int newNumEndPoints = newNumMaxBoxes * 2 + NUM_SENTINELS;

        // Allocate memory for the new boxes and end-points arrays
        BoxAABB[] newBoxesArray = new BoxAABB[newNumMaxBoxes];
        EndPoint[] newEndPointsXArray = new EndPoint[newNumEndPoints];
        EndPoint[] newEndPointsYArray = new EndPoint[newNumEndPoints];
        EndPoint[] newEndPointsZArray = new EndPoint[newNumEndPoints];

        assert (newBoxesArray != null);
        assert (newEndPointsXArray != null);
        assert (newEndPointsYArray != null);
        assert (newEndPointsZArray != null);

        // If the arrays were not empty before
        if (numBoxes > 0) {

            // Copy the data from the old arrays into the new one
            System.arraycopy(boxes, 0, newBoxesArray, 0, numBoxes);
            System.arraycopy(endPoints[0], 0, newEndPointsXArray, 0, numEndPoints);
            System.arraycopy(endPoints[1], 0, newEndPointsYArray, 0, numEndPoints);
            System.arraycopy(endPoints[2], 0, newEndPointsZArray, 0, numEndPoints);
        } else {   // If the arrays were empty

            // Add the limits endpoints (sentinels) into the array
            long min = Utils.encodeFloatIntoInteger(-Float.MAX_VALUE);
            long max = Utils.encodeFloatIntoInteger(Float.MAX_VALUE);
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
        boxes = newBoxesArray;
        endPoints[0] = newEndPointsXArray;
        endPoints[1] = newEndPointsYArray;
        endPoints[2] = newEndPointsZArray;

        numMaxBoxes = newNumMaxBoxes;
    }

    // Shrink the boxes and end-points arrays when too much memory is allocated
    protected void shrinkArrays() {

        // New number of boxes and end-points in the array
        int nextPowerOf2 = PairManager.ComputeNextPowerOfTwo((numBoxes - 1) / 100);
        int newNumMaxBoxes = (numBoxes > 100) ? nextPowerOf2 * 100 : 100;
        int numEndPoints = numBoxes * 2 + NUM_SENTINELS;
        int newNumEndPoints = newNumMaxBoxes * 2 + NUM_SENTINELS;

        assert (newNumMaxBoxes < numMaxBoxes);

        // Sort the list of the free boxes indices in ascending order
        Collections.sort(freeBoxIndices);

        // Reorganize the boxes inside the boxes array so that all the boxes are at the
        // beginning of the array
        Map<CollisionBody, Integer> newMapBodyToBoxIndex = new HashMap<>();
        for (Map.Entry it : mapBodyToBoxIndex.entrySet()) {

            CollisionBody body = (CollisionBody) it.getKey();
            int boxIndex = (int) it.getValue();

            // If the box index is outside the range of the current number of boxes
            if (boxIndex >= numBoxes) {

                assert (!freeBoxIndices.isEmpty());

                // Get a new box index for that body (from the list of free box indices)
                int newBoxIndex = freeBoxIndices.get(0);
                freeBoxIndices.remove(0);
                assert (newBoxIndex < numBoxes);

                // Copy the box to its new location in the boxes array
                BoxAABB oldBox = boxes[boxIndex];
                BoxAABB newBox = boxes[newBoxIndex];
                assert (oldBox.body.getBodyID() == body.getBodyID());
                newBox.body = oldBox.body;
                for (int axis = 0; axis < 3; axis++) {

                    // Copy the minimum and maximum end-points indices
                    newBox.min[axis] = oldBox.min[axis];
                    newBox.max[axis] = oldBox.max[axis];

                    // Update the box index of the end-points
                    EndPoint minimumEndPoint = endPoints[axis][newBox.min[axis]];
                    EndPoint maximumEndPoint = endPoints[axis][newBox.max[axis]];
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

        assert (newMapBodyToBoxIndex.size() == mapBodyToBoxIndex.size());
        mapBodyToBoxIndex.clear();
        mapBodyToBoxIndex.putAll(newMapBodyToBoxIndex);

        // Allocate memory for the new boxes and end-points arrays
        BoxAABB[] newBoxesArray = new BoxAABB[newNumMaxBoxes];
        EndPoint[] newEndPointsXArray = new EndPoint[newNumEndPoints];
        EndPoint[] newEndPointsYArray = new EndPoint[newNumEndPoints];
        EndPoint[] newEndPointsZArray = new EndPoint[newNumEndPoints];

        assert (newBoxesArray != null);
        assert (newEndPointsXArray != null);
        assert (newEndPointsYArray != null);
        assert (newEndPointsZArray != null);

        // Copy the data from the old arrays into the new one
        System.arraycopy(boxes, 0, newBoxesArray, 0, numBoxes);
        System.arraycopy(endPoints[0], 0, newEndPointsXArray, 0, numEndPoints);
        System.arraycopy(endPoints[1], 0, newEndPointsYArray, 0, numEndPoints);
        System.arraycopy(endPoints[2], 0, newEndPointsZArray, 0, numEndPoints);

        // Delete the old arrays
        // Assign the pointer to the new arrays
        boxes = newBoxesArray;
        endPoints[0] = newEndPointsXArray;
        endPoints[1] = newEndPointsYArray;
        endPoints[2] = newEndPointsZArray;

        numMaxBoxes = newNumMaxBoxes;
    }

    // Check for 1D box intersection between two boxes that are sorted on the given axis.
    // Therefore, only one test is necessary here. We know that the
    // minimum of box1 cannot be larger that the maximum of box2 on the axis.
    protected boolean testIntersect1DSortedAABBs(BoxAABB box1, AABBInt box2, EndPoint[] endPointsArray, int axis) {
        return !(endPointsArray[box1.max[axis]].value < box2.min[axis]);
    }

    // Check for 2D box intersection. This method is used when we know
    // that two boxes already overlap on one axis and when want to test
    // if they also overlap on the two others axis.
    protected boolean testIntersect2D(BoxAABB box1, BoxAABB box2, int axis1, int axis2) {
        return !(box2.max[axis1] < box1.min[axis1] || box1.max[axis1] < box2.min[axis1]
                || box2.max[axis2] < box1.min[axis2] || box1.max[axis2] < box2.min[axis2]);
    }

    // Notify the broad-phase that the AABB of an object has changed.
    // The input is an AABB with integer coordinates
    protected void updateObjectIntegerAABB(CollisionBody body, AABBInt aabbInt) {

        assert (aabbInt.min[0] > Utils.encodeFloatIntoInteger(-Float.MAX_VALUE));
        assert (aabbInt.min[1] > Utils.encodeFloatIntoInteger(-Float.MAX_VALUE));
        assert (aabbInt.min[2] > Utils.encodeFloatIntoInteger(-Float.MAX_VALUE));
        assert (aabbInt.max[0] < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (aabbInt.max[1] < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (aabbInt.max[2] < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));

        // Get the corresponding box
        int boxIndex = mapBodyToBoxIndex.get(body);
        BoxAABB box = boxes[boxIndex];

        // Current axis
        for (int axis = 0; axis < 3; axis++) {

            // Get the two others axis
            int otherAxis1 = (1 << axis) & 3;
            int otherAxis2 = (1 << otherAxis1) & 3;

            // Get the starting end-point of the current axis
            EndPoint[] startEndPointsCurrentAxis = endPoints[axis];

            // -------- Update the minimum end-point ------------//
            EndPoint currentMinEndPoint = startEndPointsCurrentAxis[box.min[axis]];
            int currentMinEndPointIndex = box.min[axis];
            assert (currentMinEndPoint.isMin);

            // Get the minimum value of the AABB on the current axis
            long limit = aabbInt.min[axis];

            // If the minimum value of the AABB is smaller
            // than the current minimum endpoint
            if (limit < currentMinEndPoint.value) {

                currentMinEndPoint.value = limit;

                // The minimum end-point is moving left
                EndPoint savedEndPoint = currentMinEndPoint;
                int indexEndPoint = currentMinEndPointIndex;
                int savedEndPointIndex = indexEndPoint;

                while ((currentMinEndPoint = startEndPointsCurrentAxis[--currentMinEndPointIndex]).value > limit) {
                    BoxAABB id1 = boxes[currentMinEndPoint.boxID];
                    boolean isMin = currentMinEndPoint.isMin;

                    // If it's a maximum end-point
                    if (!isMin) {
                        // The minimum end-point is moving to the left and
                        // passed a maximum end-point. Thus, the boxes start
                        // overlapping on the current axis. Therefore we test
                        // for box intersection
                        if (!box.equals(id1)) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)
                                    && testIntersect1DSortedAABBs(id1, aabbInt, startEndPointsCurrentAxis, axis)) {

                                // Add an overlapping pair to the pair manager
                                pairManager.addPair(body, id1.body);
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
                        boxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        boxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit > currentMinEndPoint.value) {  // The minimum of the box has moved to the right

                currentMinEndPoint.value = limit;

                // The minimum end-point is moving right
                EndPoint savedEndPoint = currentMinEndPoint;
                int indexEndPoint = currentMinEndPointIndex;
                int savedEndPointIndex = indexEndPoint;

                // For each end-point between the current position of the minimum
                // end-point and the new position of the minimum end-point
                while ((currentMinEndPoint = startEndPointsCurrentAxis[++currentMinEndPointIndex]).value < limit) {

                    BoxAABB id1 = boxes[currentMinEndPoint.boxID];
                    boolean isMin = currentMinEndPoint.isMin;

                    // If it's a maximum end-point
                    if (!isMin) {
                        // The minimum end-point is moving to the right and
                        // passed a maximum end-point. Thus, the boxes stop
                        // overlapping on the current axis.
                        if (!box.equals(id1)) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {

                                // Remove the pair from the pair manager
                                pairManager.removePair(body.getBodyID(), id1.body.getBodyID());
                            }
                        }

                        id1.max[axis] = indexEndPoint++;
                    } else {
                        id1.min[axis] = indexEndPoint++;
                    }

                    startEndPointsCurrentAxis[currentMinEndPointIndex - 1] = currentMinEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        boxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        boxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
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
                int indexEndPoint = currentMaxEndPointIndex;
                int savedEndPointIndex = indexEndPoint;

                while ((currentMaxEndPoint = startEndPointsCurrentAxis[++currentMaxEndPointIndex]).value < limit) {

                    // Get the next end-point
                    BoxAABB id1 = boxes[currentMaxEndPoint.boxID];
                    boolean isMin = currentMaxEndPoint.isMin;

                    // If it's a maximum end-point
                    if (isMin) {
                        // The maximum end-point is moving to the right and
                        // passed a minimum end-point. Thus, the boxes start
                        // overlapping on the current axis. Therefore we test
                        // for box intersection
                        if (!box.equals(id1)) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)
                                    && testIntersect1DSortedAABBs(id1, aabbInt, startEndPointsCurrentAxis, axis)) {

                                // Add an overlapping pair to the pair manager
                                pairManager.addPair(body, id1.body);
                            }
                        }

                        id1.min[axis] = indexEndPoint++;
                    } else {
                        id1.max[axis] = indexEndPoint++;
                    }

                    startEndPointsCurrentAxis[currentMaxEndPointIndex - 1] = currentMaxEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        boxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        boxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit < currentMaxEndPoint.value) {  // If the AABB is moving to the left 

                currentMaxEndPoint.value = limit;

                EndPoint savedEndPoint = currentMaxEndPoint;
                int indexEndPoint = currentMaxEndPointIndex;
                int savedEndPointIndex = indexEndPoint;

                // For each end-point between the current position of the maximum
                // end-point and the new position of the maximum end-point
                while ((currentMaxEndPoint = startEndPointsCurrentAxis[--currentMaxEndPointIndex]).value > limit) {
                    BoxAABB id1 = boxes[currentMaxEndPoint.boxID];
                    boolean isMin = currentMaxEndPoint.isMin;

                    // If it's a minimum end-point
                    if (isMin) {
                        // The maximum end-point is moving to the right and
                        // passed a minimum end-point. Thus, the boxes stop
                        // overlapping on the current axis.
                        if (!box.equals(id1)) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {

                                // Remove the pair from the pair manager
                                pairManager.removePair(body.getBodyID(), id1.body.getBodyID());
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
                        boxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        boxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            }
        }
    }

    // Notify the broad-phase about a new object in the world
    // This method adds the AABB of the object ion to broad-phase
    @Override
    public void addObject(CollisionBody body, AABB aabb) {

        int boxIndex;

        assert (Utils.encodeFloatIntoInteger(aabb.getMin().getX()) > Utils.encodeFloatIntoInteger(-Float.MAX_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMin().getY()) > Utils.encodeFloatIntoInteger(-Float.MAX_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMin().getZ()) > Utils.encodeFloatIntoInteger(-Float.MAX_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMax().getX()) < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMax().getY()) < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));
        assert (Utils.encodeFloatIntoInteger(aabb.getMax().getZ()) < Utils.encodeFloatIntoInteger(Float.MAX_VALUE));

        // If the index of the first free box is valid (means that
        // there is a bucket in the middle of the array that doesn't
        // contain a box anymore because it has been removed)
        if (!freeBoxIndices.isEmpty()) {
            int lastIndex = freeBoxIndices.size() - 1;
            boxIndex = freeBoxIndices.get(lastIndex);
            freeBoxIndices.remove(lastIndex);
        } else {
            // If the array boxes and end-points arrays are full
            if (numBoxes == numMaxBoxes) {
                // Resize the arrays to make them larger
                resizeArrays();
            }

            boxIndex = numBoxes;
        }

        // Move the maximum limit end-point two elements further
        // at the end-points array in all three axis
        int indexLimitEndPoint = 2 * numBoxes + NUM_SENTINELS - 1;
        for (int axis = 0; axis < 3; axis++) {
            EndPoint maxLimitEndPoint = endPoints[axis][indexLimitEndPoint];
            assert (endPoints[axis][0].boxID == INVALID_INDEX && endPoints[axis][0].isMin);
            assert (maxLimitEndPoint.boxID == INVALID_INDEX && !maxLimitEndPoint.isMin);
            if (endPoints[axis][indexLimitEndPoint + 2] == null) {
                endPoints[axis][indexLimitEndPoint + 2] = new EndPoint();
            }
            EndPoint newMaxLimitEndPoint = endPoints[axis][indexLimitEndPoint + 2];
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.boxID, maxLimitEndPoint.isMin,
                    maxLimitEndPoint.value);
        }

        // Create a new box
        if (boxes[boxIndex] == null) {
            boxes[boxIndex] = new BoxAABB();
        }
        BoxAABB box = boxes[boxIndex];
        box.body = body;
        long maxEndPointValue = Utils.encodeFloatIntoInteger(Float.MAX_VALUE) - 1;
        long minEndPointValue = Utils.encodeFloatIntoInteger(Float.MAX_VALUE) - 2;
        for (int axis = 0; axis < 3; axis++) {
            box.min[axis] = indexLimitEndPoint;
            box.max[axis] = indexLimitEndPoint + 1;
            EndPoint minimumEndPoint = endPoints[axis][box.min[axis]];
            minimumEndPoint.setValues(boxIndex, true, minEndPointValue);
            if (endPoints[axis][box.max[axis]] == null) {
                endPoints[axis][box.max[axis]] = new EndPoint();
            }
            EndPoint maximumEndPoint = endPoints[axis][box.max[axis]];
            maximumEndPoint.setValues(boxIndex, false, maxEndPointValue);
        }

        // Add the body pointer to box index mapping
        mapBodyToBoxIndex.put(body, boxIndex);

        numBoxes++;

        // Call the update method to put the end-points of the new AABB at the
        // correct position in the array. This will also create the overlapping
        // pairs in the pair manager if the new AABB is overlapping with others
        // AABBs
        updateObject(body, aabb);
    }

    // Notify the broad-phase about an object that has been removed from the world
    @Override
    public void removeObject(CollisionBody body) {

        assert (numBoxes > 0);

        // Call the update method with an AABB that is very far away
        // in order to remove all overlapping pairs from the pair manager
        long maxEndPointValue = Utils.encodeFloatIntoInteger(Float.MAX_VALUE) - 1;
        long minEndPointValue = Utils.encodeFloatIntoInteger(Float.MAX_VALUE) - 2;
        AABBInt aabbInt = new AABBInt(minEndPointValue, maxEndPointValue);
        updateObjectIntegerAABB(body, aabbInt);

        // Get the corresponding box
        int boxIndex = mapBodyToBoxIndex.get(body);

        // Remove the end-points of the box by moving the maximum end-points two elements back in
        // the end-points array
        int indexLimitEndPoint = 2 * numBoxes + NUM_SENTINELS - 1;
        for (int axis = 0; axis < 3; axis++) {
            EndPoint maxLimitEndPoint = endPoints[axis][indexLimitEndPoint];
            assert (endPoints[axis][0].boxID == INVALID_INDEX && endPoints[axis][0].isMin);
            assert (maxLimitEndPoint.boxID == INVALID_INDEX && !maxLimitEndPoint.isMin);
            EndPoint newMaxLimitEndPoint = endPoints[axis][indexLimitEndPoint - 2];
            assert (endPoints[axis][indexLimitEndPoint - 1].boxID == boxIndex);
            assert (!endPoints[axis][indexLimitEndPoint - 1].isMin);
            assert (newMaxLimitEndPoint.boxID == boxIndex);
            assert (newMaxLimitEndPoint.isMin);
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.boxID, maxLimitEndPoint.isMin, maxLimitEndPoint.value);
        }

        // Add the box index into the list of free indices
        freeBoxIndices.add(boxIndex);

        mapBodyToBoxIndex.remove(body);
        numBoxes--;

        // Check if we need to shrink the allocated memory
        int nextPowerOf2 = PairManager.ComputeNextPowerOfTwo((numBoxes - 1) / 100);
        if (nextPowerOf2 * 100 < numMaxBoxes) {
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
