package net.smert.jreactphysics3d.body;

/**
 * This class is an abstract class to represent a body of the physics engine.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class Body {

    // True if the body is active
    protected boolean mIsActive;

    // True if the body is allowed to go to sleep for better efficiency
    protected boolean mIsAllowedToSleep;

    // True if the body has already been added in an island (for sleeping technique)
    protected boolean mIsAlreadyInIsland;

    // True if the body is sleeping (for sleeping technique)
    protected boolean mIsSleeping;

    // Elapsed time since the body velocity was bellow the sleep velocity
    protected float mSleepTime;

    // ID of the body
    protected int mID;

    // Constructor
    public Body(int id) {
        assert (id > 0);
        mIsActive = true;
        mIsAllowedToSleep = true;
        mIsAlreadyInIsland = false;
        mIsSleeping = false;
        mSleepTime = 0;
        mID = id;
    }

    // Return true if the body is active
    public boolean isActive() {
        return mIsActive;
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

    public boolean isAlreadyInIsland() {
        return mIsAlreadyInIsland;
    }

    public void setIsAlreadyInIsland(boolean isAlreadyInIsland) {
        mIsAlreadyInIsland = isAlreadyInIsland;
    }

    // Return whether or not the body is sleeping
    public boolean isSleeping() {
        return mIsSleeping;
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

    public float getSleepTime() {
        return mSleepTime;
    }

    public void setSleepTime(float sleepTime) {
        mSleepTime = sleepTime;
    }

    // Return the id of the body
    public int getID() {
        return mID;
    }

}
