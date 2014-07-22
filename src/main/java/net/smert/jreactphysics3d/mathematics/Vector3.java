package net.smert.jreactphysics3d.mathematics;

import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class represents a 3D vector.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Vector3 {

    /// Component x
    public float x;

    /// Component y
    public float y;

    /// Component z
    public float z;

    // Constructor of the class Vector3D
    public Vector3() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    // Constructor with arguments
    public Vector3(float newX, float newY, float newZ) {
        x = newX;
        y = newY;
        z = newZ;
    }

    // Copy-constructor
    public Vector3(Vector3 vector) {
        x = vector.x;
        y = vector.y;
        z = vector.z;
    }

    // Set the vector to zero
    public void setToZero() {
        x = 0;
        y = 0;
        z = 0;
    }

    // Set all the values of the vector
    public void setAllValues(float newX, float newY, float newZ) {
        x = newX;
        y = newY;
        z = newZ;
    }

    // Return the length of the vector
    public float length() {
        return (float) Math.sqrt(x * x + y * y + z * z);
    }

    // Return the square of the length of the vector
    public float lengthSquare() {
        return x * x + y * y + z * z;
    }

    // Scalar product of two vectors (public)
    public float dot(Vector3 vector) {
        return (x * vector.x + y * vector.y + z * vector.z);
    }

    // Cross product of two vectors (public)
    public Vector3 cross(Vector3 vector) {
        return new Vector3(y * vector.z - z * vector.y,
                z * vector.x - x * vector.z,
                x * vector.y - y * vector.x);
    }

    // Normalize the vector
    public void normalize() {
        float lengthVector = length();
        assert (lengthVector > Defaults.MACHINE_EPSILON);
        float lengthInv = 1.0f / lengthVector;
        x *= lengthInv;
        y *= lengthInv;
        z *= lengthInv;
    }

    // Return the corresponding absolute value vector
    public Vector3 getAbsoluteVector() {
        return new Vector3(Math.abs(x), Math.abs(y), Math.abs(z));
    }

    // Return the axis with the minimal value
    public int getMinAxis() {
        return (x < y ? (x < z ? 0 : 2) : (y < z ? 1 : 2));
    }

    // Return the axis with the maximal value
    public int getMaxAxis() {
        return (x < y ? (y < z ? 2 : 1) : (x < z ? 2 : 0));
    }

    // Return true if the vector is unit and false otherwise
    public boolean isUnit() {
        return Mathematics.approxEqual(lengthSquare(), 1.0f, Defaults.MACHINE_EPSILON);
    }

    // Return true if the vector is the zero vector
    public boolean isZero() {
        return Mathematics.approxEqual(lengthSquare(), 0.0f, Defaults.MACHINE_EPSILON);
    }

    // Overloaded operator for the equality condition
    public boolean operatorEquals(Vector3 vector) {
        return (x == vector.x && y == vector.y && z == vector.z);
    }

    // Overloaded operator for the is different condition
    public boolean operatorNotEquals(Vector3 vector) {
        return !(operatorEquals(vector));
    }

    // Overloaded operator for addition with assignment
    public Vector3 operatorAddEqual(Vector3 vector) {
        x += vector.x;
        y += vector.y;
        z += vector.z;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Vector3 operatorSubtractEqual(Vector3 vector) {
        x -= vector.x;
        y -= vector.y;
        z -= vector.z;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public Vector3 operatorMultiplyEqual(float number) {
        x *= number;
        y *= number;
        z *= number;
        return this;
    }

    // Overloaded operator for division by a number with assignment
    public Vector3 operatorDivideEqual(float number) {
        assert (number > Defaults.MACHINE_EPSILON);
        x /= number;
        y /= number;
        z /= number;
        return this;
    }

    // Overloaded operator for value access
    public float operatorSquareBrackets(int index) {
        if (index == 0) {
            return x;
        } else if (index == 1) {
            return y;
        } else if (index == 2) {
            return z;
        }
        throw new IllegalArgumentException("Unknown index: " + index);
    }

    // Overloaded operator for addition
    public static Vector3 operatorAdd(Vector3 vector1, Vector3 vector2) {
        return new Vector3(vector1.x + vector2.x, vector1.y + vector2.y, vector1.z + vector2.z);
    }

    // Overloaded operator for substraction
    public static Vector3 operatorSubtract(Vector3 vector1, Vector3 vector2) {
        return new Vector3(vector1.x - vector2.x, vector1.y - vector2.y, vector1.z - vector2.z);
    }

    // Overloaded operator for the negative of a vector
    public static Vector3 operatorNegative(Vector3 vector) {
        return new Vector3(-vector.x, -vector.y, -vector.z);
    }

    // Overloaded operator for multiplication with a number
    public static Vector3 operatorMultiply(Vector3 vector, float number) {
        return new Vector3(number * vector.x, number * vector.y, number * vector.z);
    }

    // Overloaded operator for division by a number
    public static Vector3 operatorDivide(Vector3 vector, float number) {
        assert (number > Defaults.MACHINE_EPSILON);
        return new Vector3(vector.x / number, vector.y / number, vector.z / number);
    }

    // Overloaded operator for multiplication with a number
    public static Vector3 operatorMultiply(float number, Vector3 vector) {
        return operatorMultiply(vector, number);
    }

    // Assignment operator
    public Vector3 operatorEqual(Vector3 vector) {
        if (vector != this) {
            x = vector.x;
            y = vector.y;
            z = vector.z;
        }
        return this;
    }

    // Return the corresponding unit vector
    public Vector3 getUnit() {
        float lengthVector = length();

        assert (lengthVector > Defaults.MACHINE_EPSILON);

        // Compute and return the unit vector
        float lengthInv = 1.0f / lengthVector;
        return new Vector3(x * lengthInv, y * lengthInv, z * lengthInv);
    }

    // Return one unit orthogonal vector of the current vector
    public Vector3 getOneUnitOrthogonalVector() {

        assert (length() > Defaults.MACHINE_EPSILON);

        // Get the minimum element of the vector
        Vector3 vectorAbs = new Vector3(Math.abs(x), Math.abs(y), Math.abs(z));
        int minElement = vectorAbs.getMinAxis();

        if (minElement == 0) {
            return new Vector3(0.0f, -z, y).operatorDivideEqual((float) Math.sqrt(y * y + z * z));
        } else if (minElement == 1) {
            return new Vector3(-z, 0.0f, x).operatorDivideEqual((float) Math.sqrt(x * x + z * z));
        } else {
            return new Vector3(-y, x, 0.0f).operatorDivideEqual((float) Math.sqrt(x * x + y * y));
        }

    }

    @Override
    public int hashCode() {
        int hash = 5;
        hash = 11 * hash + Float.floatToIntBits(this.x);
        hash = 11 * hash + Float.floatToIntBits(this.y);
        hash = 11 * hash + Float.floatToIntBits(this.z);
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Vector3 other = (Vector3) obj;
        if (Float.floatToIntBits(this.x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(this.y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        return Float.floatToIntBits(this.z) == Float.floatToIntBits(other.z);
    }

    @Override
    public String toString() {
        return "(x= " + x + ", y= " + y + ", z= " + z + ")";
    }

}
