package net.smert.jreactphysics3d.configuration;

/**
 * Position correction technique used in the contact solver (for contacts)
 * 
 * BAUMGARTE_CONTACTS : Faster but can be innacurate and can lead to unexpected bounciness in some situations (due to
 * error correction factor being added to the bodies momentum).
 * 
 * SPLIT_IMPULSES : A bit slower but the error correction factor is not added to the bodies momentum. This is the option
 * used by default.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public enum ContactsPositionCorrectionTechnique {

    BAUMGARTE_CONTACTS,
    SPLIT_IMPULSES
}
