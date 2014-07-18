package net.smert.jreactphysics3d.body;

/**
 * This class is an abstract class to represent a body of the physics engine.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class Body {

    /// ID of the body
    protected int mID;

    /// True if the body has already been added in an island (for sleeping technique)
    protected boolean mIsAlreadyInIsland;

    /// True if the body is allowed to go to sleep for better efficiency
    protected boolean mIsAllowedToSleep;

    /// True if the body is active
    protected boolean mIsActive;

    /// True if the body is sleeping (for sleeping technique)
    protected boolean mIsSleeping;

    /// Elapsed time since the body velocity was bellow the sleep velocity
    protected float mSleepTime;

    /// Private copy-constructor
    protected Body(Body body) {
    }

    /// Private assignment operator
    protected Body operatorEqual(Body body) {
        return null;
    }

    // Constructor
    public Body(int id) {
        mID = id;
        mIsAlreadyInIsland = false;
        mIsAllowedToSleep = true;
        mIsActive = true;
        mIsSleeping = false;
        mSleepTime = 0;
    }

    // Return the id of the body
    public int getID() {
        return mID;
    }

    // Return whether or not the body is allowed to sleep
    public boolean isAllowedToSleep() {
        return mIsAllowedToSleep;
    }

    // Set whether or not the body is allowed to go to sleep
    public void setIsAllowedToSleep(boolean isAllowedToSleep) {
        mIsAllowedToSleep = isAllowedToSleep;

        if (!mIsAllowedToSleep) {
            setIsSleeping(false);
        }
    }

    // Return whether or not the body is sleeping
    public boolean isSleeping() {
        return mIsSleeping;
    }

    // Return true if the body is active
    public boolean isActive() {
        return mIsActive;
    }

    // Set the variable to know whether or not the body is sleeping
    public void setIsSleeping(boolean isSleeping) {

        if (isSleeping) {
            mSleepTime = 0.0f;
        } else {
            if (mIsSleeping) {
                mSleepTime = 0.0f;
            }
        }

        mIsSleeping = isSleeping;
    }

    // Smaller than operator
    public boolean operatorLessThan(Body body2) {
        return (mID < body2.mID);
    }

    // Larger than operator
    public boolean operatorGreaterThan(Body body2) {
        return (mID > body2.mID);
    }

    // Equal operator
    public boolean operatorEquals(Body body2) {
        return (mID == body2.mID);
    }

    // Not equal operator
    public boolean operatorNotEquals(Body body2) {
        return (mID != body2.mID);
    }

}
