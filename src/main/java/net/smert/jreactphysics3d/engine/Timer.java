package net.smert.jreactphysics3d.engine;

/**
 * This class will take care of the time in the physics engine. It uses functions that depend on the current platform to
 * get the current time.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Timer {

    /// Timestep dt of the physics engine (timestep > 0.0f)
    private double mTimeStep;

    /// Last time the timer has been updated
    private long mLastUpdateTime;

    /// Time difference between the two last timer update() calls
    private long mDeltaTime;

    /// Used to fix the time step and avoid strange time effects
    private double mAccumulator;

    /// True if the timer is running
    private boolean mIsRunning;

    // Constructor
    public Timer(double timeStep) {
        assert (timeStep > 0.0f);

        mTimeStep = timeStep;
        mIsRunning = false;
    }

    // Return the timestep of the physics engine
    public double getTimeStep() {
        return mTimeStep;
    }

    // Set the timestep of the physics engine
    public void setTimeStep(double timeStep) {
        assert (timeStep > 0.0f);
        mTimeStep = timeStep;
    }

    // Return the current time
    public long getPhysicsTime() {
        return mLastUpdateTime;
    }

    // Return if the timer is running
    public boolean getIsRunning() {
        return mIsRunning;
    }

    // Start the timer
    public void start() {
        if (!mIsRunning) {

            // Get the current system time
            mLastUpdateTime = getCurrentSystemTime();

            mAccumulator = 0.0;
            mIsRunning = true;
        }
    }

    // Stop the timer
    public void stop() {
        mIsRunning = false;
    }

    // True if it's possible to take a new step
    public boolean isPossibleToTakeStep() {
        return (mAccumulator >= mTimeStep);
    }

    // Take a new step => update the timer by adding the timeStep value to the current time
    public void nextStep() {
        assert (mIsRunning);

        // Update the accumulator value
        mAccumulator -= mTimeStep;
    }

    // Compute the interpolation factor
    public float computeInterpolationFactor() {
        return (float) (mAccumulator / mTimeStep);
    }

    // Compute the time since the last update() call and add it to the accumulator
    public void update() {

        // Get the current system time
        long currentTime = getCurrentSystemTime();

        // Compute the delta display time between two display frames
        mDeltaTime = currentTime - mLastUpdateTime;

        // Update the current display time
        mLastUpdateTime = currentTime;

        // Update the accumulator value
        mAccumulator += mDeltaTime;
    }

    // Return the current time of the system in seconds
    public static long getCurrentSystemTime() {
        return System.nanoTime();
    }

}
