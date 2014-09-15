package net.smert.jreactphysics3d.engine;

/**
 * It represents a profile sample in the profiler tree.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ProfileNode {

    // Name of the node
    private String mName;

    // Total number of calls of this node
    private int mNbTotalCalls;

    // Starting time of the sampling of corresponding block of code
    private float mStartingTime;

    // Total time spent in the block of code
    private float mTotalTime;

    // Recursion counter
    private int mRecursionCounter;

    // Pointer to the parent node
    private ProfileNode mParentNode;

    // Pointer to a child node
    private ProfileNode mChildNode;

    // Pointer to a sibling node
    private ProfileNode mSiblingNode;

    // Constructor
    public ProfileNode(String name, ProfileNode parentNode) {
        mName = name;
        mNbTotalCalls = 0;
        mStartingTime = 0l;
        mTotalTime = 0l;
        mRecursionCounter = 0;
        mParentNode = parentNode;
        mChildNode = null;
        mSiblingNode = null;

        reset();
    }

    // Return a pointer to the parent node
    public ProfileNode getParentNode() {
        return mParentNode;
    }

    // Return a pointer to a sibling node
    public ProfileNode getSiblingNode() {
        return mSiblingNode;
    }

    // Return a pointer to a child node
    public ProfileNode getChildNode() {
        return mChildNode;
    }

    // Return the name of the node
    public String getName() {
        return mName;
    }

    // Return the total number of call of the corresponding block of code
    public int getNumTotalCalls() {
        return mNbTotalCalls;
    }

    // Return the total time spent in the block of code
    public float getTotalTime() {
        return mTotalTime;
    }

    // Return a pointer to a sub node with a given name
    public ProfileNode findSubNode(String name) {

        // Try to find the node among the child nodes
        ProfileNode child = mChildNode;
        while (child != null) {
            if (child.mName == name) {
                return child;
            }
            child = child.mSiblingNode;
        }

        // The nose has not been found. Therefore, we create it
        // and add it to the profiler tree
        ProfileNode newNode = new ProfileNode(name, this);
        newNode.mSiblingNode = mChildNode;
        mChildNode = newNode;

        return newNode;
    }

    // Called when we enter the block of code corresponding to this profile node
    public void enterBlockOfCode() {
        mNbTotalCalls++;

        // If the current code is not called recursively
        if (mRecursionCounter == 0) {

            // Get the current system time to initialize the starting time of
            // the profiling of the current block of code
            mStartingTime = Timer.GetCurrentSystemTime() * 1000.0f;
        }

        mRecursionCounter++;
    }

    // Called when we exit the block of code corresponding to this profile node
    public boolean exitBlockOfCode() {
        mRecursionCounter--;

        if (mRecursionCounter == 0 && mNbTotalCalls != 0) {

            // Get the current system time
            float currentTime = Timer.GetCurrentSystemTime() * 1000.0f;

            // Increase the total elasped time in the current block of code
            mTotalTime += currentTime - mStartingTime;
        }

        // Return true if the current code is not recursing
        return (mRecursionCounter == 0);
    }

    // Reset the profiling of the node
    public void reset() {
        mNbTotalCalls = 0;
        mTotalTime = 0l;

        // Reset the child node
        if (mChildNode != null) {
            mChildNode.reset();
        }

        // Reset the sibling node
        if (mSiblingNode != null) {
            mSiblingNode.reset();
        }
    }

    // Destroy the node
    public void destroy() {
        //delete mChildNode;
        mChildNode = null;
        //delete mSiblingNode;
        mSiblingNode = null;
    }

}
