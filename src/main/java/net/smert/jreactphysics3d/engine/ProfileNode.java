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
package net.smert.jreactphysics3d.engine;

/**
 * It represents a profile sample in the profiler tree.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ProfileNode {

    // Starting time of the sampling of corresponding block of code
    private float startingTime;

    // Total time spent in the block of code
    private float totalTime;

    // Total number of calls of this node
    private int numTotalCalls;

    // Recursion counter
    private int recursionCounter;

    // Pointer to a child node
    private ProfileNode childNode;

    // Pointer to the parent node
    private final ProfileNode parentNode;

    // Pointer to a sibling node
    private ProfileNode siblingNode;

    // Name of the node
    private final String name;

    // Constructor
    public ProfileNode(String name, ProfileNode parentNode) {
        assert (name != null);
        startingTime = 0l;
        totalTime = 0l;
        numTotalCalls = 0;
        recursionCounter = 0;
        childNode = null;
        this.parentNode = parentNode;
        siblingNode = null;
        this.name = name;
        reset();
    }

    // Destroy the node
    public void destroy() {
        //delete mChildNode;
        childNode = null;
        //delete mSiblingNode;
        siblingNode = null;
    }

    // Called when we enter the block of code corresponding to this profile node
    public void enterBlockOfCode() {
        numTotalCalls++;

        // If the current code is not called recursively
        if (recursionCounter == 0) {

            // Get the current system time to initialize the starting time of
            // the profiling of the current block of code
            startingTime = Timer.GetCurrentSystemTime() * 1000.0f;
        }

        recursionCounter++;
    }

    // Called when we exit the block of code corresponding to this profile node
    public boolean exitBlockOfCode() {
        recursionCounter--;

        if (recursionCounter == 0 && numTotalCalls != 0) {

            // Get the current system time
            float currentTime = Timer.GetCurrentSystemTime() * 1000.0f;

            // Increase the total elasped time in the current block of code
            totalTime += currentTime - startingTime;
        }

        // Return true if the current code is not recursing
        return (recursionCounter == 0);
    }

    // Return a pointer to a sub node with a given name
    public ProfileNode findSubNode(String name) {

        // Try to find the node among the child nodes
        ProfileNode child = childNode;
        while (child != null) {
            if (child.name.equals(name)) {
                return child;
            }
            child = child.siblingNode;
        }

        // The nose has not been found. Therefore, we create it
        // and add it to the profiler tree
        ProfileNode newNode = new ProfileNode(name, this);
        newNode.siblingNode = childNode;
        childNode = newNode;

        return newNode;
    }

    // Return the total time spent in the block of code
    public float getTotalTime() {
        return totalTime;
    }

    // Return the total number of call of the corresponding block of code
    public int getNumTotalCalls() {
        return numTotalCalls;
    }

    // Return a pointer to a child node
    public ProfileNode getChildNode() {
        return childNode;
    }

    // Return a pointer to the parent node
    public ProfileNode getParentNode() {
        return parentNode;
    }

    // Return a pointer to a sibling node
    public ProfileNode getSiblingNode() {
        return siblingNode;
    }

    // Return the name of the node
    public String getName() {
        return name;
    }

    // Reset the profiling of the node
    public final void reset() {
        totalTime = 0;
        numTotalCalls = 0;

        // Reset the child node
        if (childNode != null) {
            childNode.reset();
        }

        // Reset the sibling node
        if (siblingNode != null) {
            siblingNode.reset();
        }
    }

}
