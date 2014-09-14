package net.smert.jreactphysics3d.engine;

/**
 * This structure represents a single element of a linked list of contact manifolds
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactManifoldListElement {

    // Pointer to the actual contact manifold
    private final ContactManifold contactManifold;

    // Next element of the list
    private final ContactManifoldListElement next;

    // Constructor
    public ContactManifoldListElement(ContactManifold contactManifold, ContactManifoldListElement next) {
        this.contactManifold = contactManifold;
        this.next = next;
    }

    public ContactManifold getContactManifold() {
        return contactManifold;
    }

    public ContactManifoldListElement getNext() {
        return next;
    }

}
