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
package net.smert.jreactphysics3d.configuration;

/**
 * Position correction technique used in the contact solver (for contacts)
 *
 * BAUMGARTE_CONTACTS : Faster but can be inaccurate and can lead to unexpected bounciness in some situations (due to
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
