package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This is the main class of the profiler. This profiler is based on "Real-Time Hierarchical Profiling" article from
 * "Game Programming Gems 3" by Greg Hjelstrom and Byon Garrabrant.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Profiler {

    // Root node of the profiler tree
    private static ProfileNode mRootNode;

    // Current node in the current execution
    private static ProfileNode mCurrentNode;

    // Frame counter
    private static int mFrameCounter;

    // Starting profiling time
    private static float mProfilingStartTime;

    // Recursively print the report of a given node of the profiler tree
    private static void printRecursiveNodeReport(ProfileNodeIterator iterator, int spacing, int outputStream) {
        iterator.first();

        // If we are at the end of a branch in the profiler tree
        if (iterator.isEnd()) {
            return;
        }

        float parentTime = iterator.isRoot() ? getElapsedTimeSinceStart() : iterator.getCurrentParentTotalTime();
        long accumulatedTime = 0l;
        int nbFrames = getNbFrames();
        for (int i = 0; i < spacing; i++) {
            System.out.print(" ");
        }
        System.out.println("---------------");
        for (int i = 0; i < spacing; i++) {
            System.out.print(" ");
        }
        System.out.print("| Profiling : ");
        System.out.print(iterator.getCurrentParentName());
        System.out.print(" (total running time : ");
        System.out.print(parentTime);
        System.out.println(" ms) ---");
        long totalTime = 0l;

        // Recurse over the children of the current node
        int nbChildren = 0;
        for (int i = 0; !iterator.isEnd(); i++, iterator.next()) {
            nbChildren++;
            float currentTotalTime = iterator.getCurrentTotalTime();
            accumulatedTime += currentTotalTime;
            long fraction = parentTime > Defaults.MACHINE_EPSILON ? (long) (currentTotalTime / parentTime) * 100l : 0l;
            for (int j = 0; j < spacing; j++) {
                System.out.print(" ");
            }
            System.out.print("|   ");
            System.out.print(i);
            System.out.print(" -- ");
            System.out.print(iterator.getCurrentName());
            System.out.print(" : ");
            System.out.print(fraction);
            System.out.print(" % | ");
            System.out.print(currentTotalTime / (long) nbFrames);
            System.out.print(" ms/frame (");
            System.out.print(iterator.getCurrentNbTotalCalls());
            System.out.println(" calls)");
            totalTime += currentTotalTime;
        }

        if (parentTime < accumulatedTime) {
            System.out.println("Something is wrong !");
        }
        for (int i = 0; i < spacing; i++) {
            System.out.print(" ");
        }
        long percentage = parentTime > Defaults.MACHINE_EPSILON ? (long) ((parentTime - accumulatedTime) / parentTime) * 100l : 0l;
        float difference = parentTime - accumulatedTime;
        System.out.print("| Unaccounted : ");
        System.out.print(difference);
        System.out.print(" ms (");
        System.out.print(percentage);
        System.out.println(" %)");

        for (int i = 0; i < nbChildren; i++) {
            iterator.enterChild(i);
            printRecursiveNodeReport(iterator, spacing + 3, outputStream);
            iterator.enterParent();
        }
    }

    // Return the number of frames
    public static int getNbFrames() {
        return mFrameCounter;
    }

    // Return the elasped time since the start/reset of the profiling
    public static float getElapsedTimeSinceStart() {
        float currentTime = Timer.GetCurrentSystemTime() * 1000l;
        return currentTime - mProfilingStartTime;
    }

    // Increment the frame counter
    public static void incrementFrameCounter() {
        mFrameCounter++;
    }

    // Return an iterator over the profiler tree starting at the root
    public static ProfileNodeIterator getIterator() {
        return new ProfileNodeIterator(mRootNode);
    }

    // Destroy a previously allocated iterator
    public static void destroyIterator(ProfileNodeIterator iterator) {
        //delete iterator;
    }

    // Destroy the profiler (release the memory)
    public static void destroy() {
        mRootNode.destroy();
    }

    // Method called when we want to start profiling a block of code.
    public static void startProfilingBlock(String name) {

        // Look for the node in the tree that corresponds to the block of
        // code to profile
        if (!name.equals(mCurrentNode.getName())) {
            mCurrentNode = mCurrentNode.findSubNode(name);
        }

        // Start profile the node
        mCurrentNode.enterBlockOfCode();
    }

    // Method called at the end of the scope where the
    // startProfilingBlock() method has been called.
    public static void stopProfilingBlock() {

        // Go to the parent node unless if the current block
        // of code is recursing
        if (mCurrentNode.exitBlockOfCode()) {
            mCurrentNode = mCurrentNode.getParentNode();
        }
    }

    // Reset the timing data of the profiler (but not the profiler tree structure)
    public static void reset() {
        mRootNode.reset();
        mRootNode.enterBlockOfCode();
        mFrameCounter = 0;
        mProfilingStartTime = Timer.GetCurrentSystemTime() * 1000.0f;
    }

    // Print the report of the profiler in a given output stream
    public static void printReport(int outputStream) {
        ProfileNodeIterator iterator = getIterator();

        // Recursively print the report of each node of the profiler tree
        printRecursiveNodeReport(iterator, 0, outputStream);

        // Destroy the iterator
        destroyIterator(iterator);
    }

    static {
        // Initialization of static variables
        mRootNode = new ProfileNode("Root", null);
        mCurrentNode = mRootNode;
        mProfilingStartTime = Timer.GetCurrentSystemTime() * 1000.0f;
        mFrameCounter = 0;
    }

}
