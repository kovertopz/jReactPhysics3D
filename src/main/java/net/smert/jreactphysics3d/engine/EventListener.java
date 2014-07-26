package net.smert.jreactphysics3d.engine;

import net.smert.jreactphysics3d.constraint.ContactPointInfo;

/**
 * This class can be used to receive event callbacks from the physics engine. In order to receive callbacks, you need to
 * create a new class that inherits from this one and you must override the methods you need. Then, you need to register
 * your new event listener class to the physics world using the DynamicsWorld::setEventListener() method.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class EventListener {

    // Called when a new contact point is found between two bodies that were separated before
    public abstract void beginContact(ContactPointInfo contact);

    // Called when a new contact point is found between two bodies
    public abstract void newContact(ContactPointInfo contact);

}
