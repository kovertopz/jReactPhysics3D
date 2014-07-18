package net.smert.jreactphysics3d.configuration;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Defaults {

    /// Smallest decimal value (negative)
    public static final float DECIMAL_SMALLEST = Float.MIN_VALUE;

    /// Maximum decimal value
    public static final float DECIMAL_LARGEST = Float.MAX_VALUE;

    /// Machine epsilon
    public static final float MACHINE_EPSILON = 0.000001f;

    /// Pi constant
    public static final float PI = 3.14159265f;

    /// 2*Pi constant
    public static final float PI_TIMES_2 = 6.28318530f;

    /// Default internal constant timestep in seconds
    public static final float DEFAULT_TIMESTEP = 1.0f / 60.0f;

    /// Default friction coefficient for a rigid body
    public static final float DEFAULT_FRICTION_COEFFICIENT = 0.3f;

    /// Default bounciness factor for a rigid body
    public static final float DEFAULT_BOUNCINESS = 0.5f;

    /// True if the spleeping technique is enabled
    public static final boolean SPLEEPING_ENABLED = true;

    /// Object margin for collision detection in meters (for the GJK-EPA Algorithm)
    public static final float OBJECT_MARGIN = 0.04f;

    /// Distance threshold for two contact points for a valid persistent contact (in meters)
    public static final float PERSISTENT_CONTACT_DIST_THRESHOLD = 0.03f;

    /// Velocity threshold for contact velocity restitution
    public static final float RESTITUTION_VELOCITY_THRESHOLD = 1.0f;

    /// Number of iterations when solving the velocity constraints of the Sequential Impulse technique
    public static final int DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS = 10;

    /// Number of iterations when solving the position constraints of the Sequential Impulse technique
    public static final int DEFAULT_POSITION_SOLVER_NB_ITERATIONS = 5;

    /// Time (in seconds) that a body must stay still to be considered sleeping
    public static final float DEFAULT_TIME_BEFORE_SLEEP = 1.0f;

    /// A body with a linear velocity smaller than the sleep linear velocity (in m/s)
    /// might enter sleeping mode.
    public static final float DEFAULT_SLEEP_LINEAR_VELOCITY = 0.02f;

    /// A body with angular velocity smaller than the sleep angular velocity (in rad/s)
    /// might enter sleeping mode
    public static final float DEFAULT_SLEEP_ANGULAR_VELOCITY = 3.0f * (PI / 180.0f);

}
