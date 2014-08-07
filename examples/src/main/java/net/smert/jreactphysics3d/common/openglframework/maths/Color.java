package net.smert.jreactphysics3d.common.openglframework.maths;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Color {

    // RGBA color components
    public float r, g, b, a;

    // Constructor
    public Color() {
        this(1, 1, 1, 1);
    }

    // Constructor
    public Color(float r, float g, float b, float a) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
    }

    // Copy- ructor
    public Color(Color color) {
        r = color.r;
        g = color.g;
        b = color.b;
        a = color.a;
    }

    // Return the black color
    public static Color black() {
        return new Color(0.0f, 0.0f, 0.0f, 1.0f);
    }

    // Return the white color
    public static Color white() {
        return new Color(1.0f, 1.0f, 1.0f, 1.0f);
    }

    // = operator
    public Color operatorEqual(Color color) {
        if (color != this) {
            r = color.r;
            g = color.g;
            b = color.b;
            a = color.a;
        }
        return this;
    }

}
