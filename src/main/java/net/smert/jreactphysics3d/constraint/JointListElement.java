package net.smert.jreactphysics3d.constraint;

/**
 * This structure represents a single element of a linked list of joints
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class JointListElement {

    // Pointer to the actual joint
    public Joint joint;

    // Next element of the list
    public JointListElement next;

    // Constructor
    public JointListElement(Joint initJoint, JointListElement initNext) {
        joint = initJoint;
        next = initNext;
    }

}
