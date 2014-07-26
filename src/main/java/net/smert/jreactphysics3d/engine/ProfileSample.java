package net.smert.jreactphysics3d.engine;

/**
 * This class is used to represent a profile sample. It is constructed at the beginning of a code block we want to
 * profile and destructed at the end of the scope to profile.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ProfileSample {

    // Constructor
    public ProfileSample(String name) {

        // Ask the profiler to start profiling a block of code
        Profiler.startProfilingBlock(name);
    }

}
