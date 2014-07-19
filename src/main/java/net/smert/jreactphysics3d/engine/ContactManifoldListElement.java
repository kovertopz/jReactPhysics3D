package net.smert.jreactphysics3d.engine;

/**
 * This structure represents a single element of a linked list of contact manifolds
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactManifoldListElement {

    /// Pointer to the actual contact manifold
    public ContactManifold contactManifold;

    /// Next element of the list
    public ContactManifoldListElement next;

    /// Constructor
    public ContactManifoldListElement(ContactManifold initContactManifold, ContactManifoldListElement initNext) {
        contactManifold = initContactManifold;
        next = initNext;
    }

}
