package net.smert.jreactphysics3d.body;

/**
 * This class is an abstract class to represent a body of the physics engine.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class Body {

    // True if the body is active
    protected boolean isActive;

    // True if the body is allowed to go to sleep for better efficiency
    protected boolean isAllowedToSleep;

    // True if the body has already been added in an island (for sleeping technique)
    protected boolean isAlreadyInIsland;

    // True if the body is sleeping (for sleeping technique)
    protected boolean isSleeping;

    // Elapsed time since the body velocity was bellow the sleep velocity
    protected float sleepTime;

    // Unique ID of the body
    protected int bodyID;

    // Constructor
    public Body(int bodyID) {
        assert (bodyID >= 0);
        isActive = true;
        isAllowedToSleep = true;
        isAlreadyInIsland = false;
        isSleeping = false;
        sleepTime = 0;
        this.bodyID = bodyID;
    }

    // Return true if the body is active
    public boolean isActive() {
        return isActive;
    }

    // Return whether or not the body is allowed to sleep
    public boolean isAllowedToSleep() {
        return isAllowedToSleep;
    }

    // Set whether or not the body is allowed to go to sleep
    public void setIsAllowedToSleep(boolean isAllowedToSleep) {
        this.isAllowedToSleep = isAllowedToSleep;

        if (!this.isAllowedToSleep) {
            setIsSleeping(false);
        }
    }

    public boolean isAlreadyInIsland() {
        return isAlreadyInIsland;
    }

    public void setIsAlreadyInIsland(boolean isAlreadyInIsland) {
        this.isAlreadyInIsland = isAlreadyInIsland;
    }

    // Return whether or not the body is sleeping
    public boolean isSleeping() {
        return isSleeping;
    }

    // Set the variable to know whether or not the body is sleeping
    public void setIsSleeping(boolean isSleeping) {

        if (isSleeping) {
            sleepTime = 0.0f;
        } else {
            if (this.isSleeping) {
                sleepTime = 0.0f;
            }
        }

        this.isSleeping = isSleeping;
    }

    public float getSleepTime() {
        return sleepTime;
    }

    public void setSleepTime(float sleepTime) {
        this.sleepTime = sleepTime;
    }

    // Return the id of the body
    public int getBodyID() {
        return bodyID;
    }

}
