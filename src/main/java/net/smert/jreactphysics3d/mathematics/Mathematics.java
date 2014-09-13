package net.smert.jreactphysics3d.mathematics;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Mathematics {

    private Mathematics() {
    }

    // Function to test if two real numbers are (almost) equal
    // We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
    public static boolean ApproxEqual(float a, float b, float epsilon) {
        float difference = a - b;
        return (difference < epsilon && difference > -epsilon);
    }

    public static float ArcCos(float radians) {
        return (float) StrictMath.acos(radians);
    }

    public static float ArcSin(float radians) {
        return (float) StrictMath.asin(radians);
    }

    public static float ArcTan2(float a, float b) {
        return (float) StrictMath.atan2(a, b);
    }

    // Function that returns the result of the "value" clamped by
    // two others values "lowerLimit" and "upperLimit"
    public static float Clamp(float value, float lowerLimit, float upperLimit) {
        assert (lowerLimit <= upperLimit);
        return Math.min(Math.max(value, lowerLimit), upperLimit);
    }

    public static float Cos(float radians) {
        return (float) StrictMath.cos(radians);
    }

    public static float Sin(float radians) {
        return (float) StrictMath.sin(radians);
    }

    public static float Sqrt(float a) {
        return (float) StrictMath.sqrt(a);
    }

    public static float Tan(float radians) {
        return (float) StrictMath.tan(radians);
    }

}
