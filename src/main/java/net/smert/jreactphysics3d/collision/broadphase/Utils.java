package net.smert.jreactphysics3d.collision.broadphase;

/**
 * Utility functions used by several classes.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Utils {

    // Encode a floating value into a integer value in order to
    // work with integer comparison in the Sweep-And-Prune algorithm
    // because it is faster. The main issue when encoding floating
    // number into integer is to keep to sorting order. This is a
    // problem for negative float number. This article describes
    // how to solve this issue : http://www.stereopsis.com/radix.html
    public static long encodeFloatIntoInteger(float number) {
        long intNumber = (long) Float.floatToIntBits(number) & 0xFFFFFFFFl;

        // If it's a negative number
        if ((intNumber & 0x80000000l) == 0x80000000l) {
            intNumber = ~intNumber & 0xFFFFFFFFl;
        } else {     // If it is a positive number
            intNumber |= 0x80000000l;
        }

        return intNumber;
    }

}
