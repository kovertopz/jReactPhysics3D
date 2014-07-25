package net.smert.jreactphysics3d.engine;

/**
 * This class allows us to iterator over the profiler tree.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ProfileNodeIterator {

    /// Current parent node
    private ProfileNode mCurrentParentNode;

    /// Current child node
    private ProfileNode mCurrentChildNode;

    // Constructor
    public ProfileNodeIterator(ProfileNode startingNode) {
        mCurrentParentNode = startingNode;
        mCurrentChildNode = mCurrentParentNode.getChildNode();
    }

    // Return true if we are at the root of the profiler tree
    public boolean isRoot() {
        return (mCurrentParentNode.getParentNode() == null);
    }

    // Return true if we are at the end of a branch of the profiler tree
    public boolean isEnd() {
        return (mCurrentChildNode == null);
    }

    // Return the name of the current node
    public String getCurrentName() {
        return mCurrentChildNode.getName();
    }

    // Return the total time of the current node
    public float getCurrentTotalTime() {
        return mCurrentChildNode.getTotalTime();
    }

    // Return the total number of calls of the current node
    public int getCurrentNbTotalCalls() {
        return mCurrentChildNode.getNbTotalCalls();
    }

    // Return the name of the current parent node
    public String getCurrentParentName() {
        return mCurrentParentNode.getName();
    }

    // Return the total time of the current parent node
    public float getCurrentParentTotalTime() {
        return mCurrentParentNode.getTotalTime();
    }

    // Return the total number of calls of the current parent node
    public int getCurrentParentNbTotalCalls() {
        return mCurrentParentNode.getNbTotalCalls();
    }

    // Go to the first node
    public void first() {
        mCurrentChildNode = mCurrentParentNode.getChildNode();
    }

    // Go to the next node
    public void next() {
        mCurrentChildNode = mCurrentChildNode.getSiblingNode();
    }

    // Enter a given child node
    public void enterChild(int index) {
        mCurrentChildNode = mCurrentParentNode.getChildNode();
        while ((mCurrentChildNode != null) && (index != 0)) {
            index--;
            mCurrentChildNode = mCurrentChildNode.getSiblingNode();
        }

        if (mCurrentChildNode != null) {
            mCurrentParentNode = mCurrentChildNode;
            mCurrentChildNode = mCurrentParentNode.getChildNode();
        }
    }

    // Enter a given parent node
    public void enterParent() {
        if (mCurrentParentNode.getParentNode() != null) {
            mCurrentParentNode = mCurrentParentNode.getParentNode();
        }
        mCurrentChildNode = mCurrentParentNode.getChildNode();
    }

}
