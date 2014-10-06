/*
 * ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/
 * Copyright (c) 2010-2013 Daniel Chappuis
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim
 *    that you wrote the original software. If you use this software in a
 *    product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * This file has been modified during the port to Java and differ from the source versions.
 */
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
