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
 * This class allows us to iterator over the profiler tree.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ProfileNodeIterator {

    // Current child node
    private ProfileNode currentChildNode;

    // Current parent node
    private ProfileNode currentParentNode;

    // Constructor
    public ProfileNodeIterator(ProfileNode startingNode) {
        currentChildNode = startingNode.getChildNode();
        currentParentNode = startingNode;
    }

    // Enter a given child node
    public void enterChild(int index) {
        currentChildNode = currentParentNode.getChildNode();
        while ((currentChildNode != null) && (index != 0)) {
            index--;
            currentChildNode = currentChildNode.getSiblingNode();
        }

        if (currentChildNode != null) {
            currentParentNode = currentChildNode;
            currentChildNode = currentParentNode.getChildNode();
        }
    }

    // Enter a given parent node
    public void enterParent() {
        if (currentParentNode.getParentNode() != null) {
            currentParentNode = currentParentNode.getParentNode();
        }
        currentChildNode = currentParentNode.getChildNode();
    }

    // Go to the first node
    public void first() {
        currentChildNode = currentParentNode.getChildNode();
    }

    // Return the total time of the current node
    public float getCurrentTotalTime() {
        return currentChildNode.getTotalTime();
    }

    // Return the total time of the current parent node
    public float getCurrentParentTotalTime() {
        return currentParentNode.getTotalTime();
    }

    // Return the total number of calls of the current node
    public int getCurrentNumTotalCalls() {
        return currentChildNode.getNumTotalCalls();
    }

    // Return the total number of calls of the current parent node
    public int getCurrentParentNumTotalCalls() {
        return currentParentNode.getNumTotalCalls();
    }

    // Return the name of the current node
    public String getCurrentName() {
        return currentChildNode.getName();
    }

    // Return the name of the current parent node
    public String getCurrentParentName() {
        return currentParentNode.getName();
    }

    // Return true if we are at the end of a branch of the profiler tree
    public boolean isEnd() {
        return (currentChildNode == null);
    }

    // Return true if we are at the root of the profiler tree
    public boolean isRoot() {
        return (currentParentNode.getParentNode() == null);
    }

    // Go to the next node
    public void next() {
        currentChildNode = currentChildNode.getSiblingNode();
    }

}
