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
 * This class will take care of the time in the physics engine. It uses functions that depend on the current platform to
 * get the current time.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Timer {

    // True if the timer is running
    private boolean isRunning;

    // Used to fix the time step and avoid strange time effects
    private double accumulator;

    // Timestep dt of the physics engine (timestep > 0.0f)
    private double timeStep;

    // Time difference between the two last timer update() calls
    private float deltaTime;

    // Last time the timer has been updated
    private float lastUpdateTime;

    // Constructor
    public Timer(double timeStep) {
        assert (timeStep > 0.0f);
        isRunning = false;
        this.timeStep = timeStep;
    }

    // Compute the interpolation factor
    public float computeInterpolationFactor() {
        return (float) (accumulator / timeStep);
    }

    // Return if the timer is running
    public boolean getIsRunning() {
        return isRunning;
    }

    // Return the timestep of the physics engine
    public double getTimeStep() {
        return timeStep;
    }

    // Set the timestep of the physics engine
    public void setTimeStep(double timeStep) {
        assert (timeStep > 0.0f);
        this.timeStep = timeStep;
    }

    // Return the current time
    public float getPhysicsTime() {
        return lastUpdateTime;
    }

    // True if it's possible to take a new step
    public boolean isPossibleToTakeStep() {
        return (accumulator >= timeStep);
    }

    // Take a new step => update the timer by adding the timeStep value to the current time
    public void nextStep() {
        assert (isRunning);

        // Update the accumulator value
        accumulator -= timeStep;
    }

    // Start the timer
    public void start() {
        if (!isRunning) {

            isRunning = true;
            accumulator = 0.0;
            // Get the current system time
            lastUpdateTime = GetCurrentSystemTime();
        }
    }

    // Stop the timer
    public void stop() {
        isRunning = false;
    }

    // Compute the time since the last update() call and add it to the accumulator
    public void update() {

        // Get the current system time
        float currentTime = GetCurrentSystemTime();

        // Compute the delta display time between two display frames
        deltaTime = currentTime - lastUpdateTime;

        // Update the current display time
        lastUpdateTime = currentTime;

        // Update the accumulator value
        accumulator += deltaTime;
    }

    // Return the current time of the system in seconds
    public static float GetCurrentSystemTime() {
        return System.nanoTime() / 1000000000.0f;
    }

}
