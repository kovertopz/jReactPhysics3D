package net.smert.jreactphysics3d.configuration;

/**
 * Position correction technique used in the constraint solver (for joints).
 * BAUMGARTE_JOINTS : Faster but can be innacurate in some situations.
 * NON_LINEAR_GAUSS_SEIDEL : Slower but more precise. This is the option used by
 * default.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public enum JointsPositionCorrectionTechnique {

    BAUMGARTE_JOINTS,
    NON_LINEAR_GAUSS_SEIDEL
}
