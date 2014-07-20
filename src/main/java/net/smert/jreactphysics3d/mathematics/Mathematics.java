package net.smert.jreactphysics3d.mathematics;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Mathematics {

    /// Function to test if two real numbers are (almost) equal
    /// We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
    public static boolean approxEqual(float a, float b, float epsilon) {

        float difference = a - b;
        return (difference < epsilon && difference > -epsilon);
    }

    /// Function that returns the result of the "value" clamped by
    /// two others values "lowerLimit" and "upperLimit"
    public static float clamp(float value, float lowerLimit, float upperLimit) {
        assert (lowerLimit <= upperLimit);
        return Math.min(Math.max(value, lowerLimit), upperLimit);
    }

}
