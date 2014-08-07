package net.smert.jreactphysics3d.common.openglframework;

import net.smert.jreactphysics3d.common.openglframework.maths.Color;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class VertexData {

    /// Vertex position
    public Vector3 position;

    /// Vertex normal
    public Vector3 normal;

    // Vertex color
    public Color color;

    public VertexData(Vector3 position, Vector3 normal, Color color) {
        this.position = position;
        this.normal = normal;
        this.color = color;
    }

}
