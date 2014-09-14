package net.smert.jreactphysics3d.collision.narrowphase.GJK;

import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a simplex which is a set of 3D points. This class is used in the GJK algorithm. This
 * implementation is based on the implementation discussed in the book "Collision Detection in 3D Environments". This
 * class implements the Johnson's algorithm for computing the point of a simplex that is closest to the origin and also
 * the smallest simplex needed to represent that closest point.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Simplex {

    // Maximum length of pointsLengthSquare[i]
    private float mMaxLengthSquare;

    // pointsLengthSquare[i] = (points[i].length)^2
    private final float[] mPointsLengthSquare = new float[4];

    // Cached determinant values
    private final float[][] mDet = new float[16][4];

    // norm[i][j] = (diff[i][j].length())^2
    private final float[][] mNormSquare = new float[4][4];

    // allBits = bitsCurrentSimplex | lastFoundBit;
    private int mAllBits;

    // 4 bits that identify the current points of the simplex
    // For instance, 0101 means that points[1] and points[3] are in the simplex
    private int mBitsCurrentSimplex;

    // Number between 1 and 4 that identify the last found support point
    private int mLastFound;

    // Position of the last found support point (lastFoundBit = 0x1 << lastFound)
    private int mLastFoundBit;

    // Current points
    private final Vector3[] mPoints = new Vector3[4];

    // Support points of object A in local coordinates
    private final Vector3[] mSuppPointsA = new Vector3[4];

    // Support points of object B in local coordinates
    private final Vector3[] mSuppPointsB = new Vector3[4];

    // diff[i][j] contains points[i] - points[j]
    private final Vector3[][] mDiffLength = new Vector3[4][4];

    public Simplex() {
        mAllBits = 0;
        mBitsCurrentSimplex = 0;

        for (int i = 0; i < mPoints.length; i++) {
            mPoints[i] = new Vector3();
        }
        for (int i = 0; i < mSuppPointsA.length; i++) {
            mSuppPointsA[i] = new Vector3();
        }
        for (int i = 0; i < mSuppPointsB.length; i++) {
            mSuppPointsB[i] = new Vector3();
        }
    }

    // Return the closest point "v" in the convex hull of the points in the subset
    // represented by the bits "subset"
    private Vector3 computeClosestPointForSubset(int subset) {

        float deltaX = 0.0f;            // deltaX = sum of all det[subset][i]
        mMaxLengthSquare = 0.0f;
        Vector3 v = new Vector3();      // Closet point v = sum(lambda_i * points[i])

        // For each four point in the possible simplex set
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            // If the current point is in the subset
            if (overlap(subset, bit)) {
                // deltaX = sum of all det[subset][i]
                deltaX += mDet[subset][i];

                if (mMaxLengthSquare < mPointsLengthSquare[i]) {
                    mMaxLengthSquare = mPointsLengthSquare[i];
                }

                // Closest point v = sum(lambda_i * points[i])
                v.add(new Vector3(mPoints[i]).multiply(mDet[subset][i]));
            }
        }

        assert (deltaX > 0.0f);

        // Return the closet point "v" in the convex hull for the given subset
        return new Vector3(v).multiply(1.0f / deltaX);
    }

    // Compute the cached determinant values
    private void computeDeterminants() {

        mDet[mLastFoundBit][mLastFound] = 1.0f;

        // If the current simplex is not empty
        if (!isEmpty()) {

            // For each possible four points in the simplex set
            for (int i = 0, bitI = 0x1; i < 4; i++, bitI <<= 1) {

                // If the current point is in the simplex
                if (overlap(mBitsCurrentSimplex, bitI)) {
                    int bit2 = bitI | mLastFoundBit;

                    mDet[bit2][i] = mDiffLength[mLastFound][i].dot(mPoints[mLastFound]);
                    mDet[bit2][mLastFound] = mDiffLength[i][mLastFound].dot(mPoints[i]);

                    for (int j = 0, k, bitJ = 0x1; j < i; j++, bitJ <<= 1) {
                        if (overlap(mBitsCurrentSimplex, bitJ)) {
                            int bit3 = bitJ | bit2;

                            k = mNormSquare[i][j] < mNormSquare[mLastFound][j] ? i : mLastFound;
                            mDet[bit3][j] = mDet[bit2][i] * mDiffLength[k][j].dot(mPoints[i])
                                    + mDet[bit2][mLastFound]
                                    * mDiffLength[k][j].dot(mPoints[mLastFound]);

                            k = mNormSquare[j][i] < mNormSquare[mLastFound][i] ? j : mLastFound;
                            mDet[bit3][i] = mDet[bitJ | mLastFoundBit][j]
                                    * mDiffLength[k][i].dot(mPoints[j])
                                    + mDet[bitJ | mLastFoundBit][mLastFound]
                                    * mDiffLength[k][i].dot(mPoints[mLastFound]);

                            k = mNormSquare[i][mLastFound] < mNormSquare[j][mLastFound] ? i : j;
                            mDet[bit3][mLastFound] = mDet[bitJ | bitI][j]
                                    * mDiffLength[k][mLastFound].dot(mPoints[j])
                                    + mDet[bitJ | bitI][i]
                                    * mDiffLength[k][mLastFound].dot(mPoints[i]);
                        }
                    }
                }
            }

            if (mAllBits == 0xf) {
                int k;

                k = mNormSquare[1][0] < mNormSquare[2][0]
                        ? (mNormSquare[1][0] < mNormSquare[3][0] ? 1 : 3)
                        : (mNormSquare[2][0] < mNormSquare[3][0] ? 2 : 3);
                mDet[0xf][0] = mDet[0xe][1] * mDiffLength[k][0].dot(mPoints[1])
                        + mDet[0xe][2] * mDiffLength[k][0].dot(mPoints[2])
                        + mDet[0xe][3] * mDiffLength[k][0].dot(mPoints[3]);

                k = mNormSquare[0][1] < mNormSquare[2][1]
                        ? (mNormSquare[0][1] < mNormSquare[3][1] ? 0 : 3)
                        : (mNormSquare[2][1] < mNormSquare[3][1] ? 2 : 3);
                mDet[0xf][1] = mDet[0xd][0] * mDiffLength[k][1].dot(mPoints[0])
                        + mDet[0xd][2] * mDiffLength[k][1].dot(mPoints[2])
                        + mDet[0xd][3] * mDiffLength[k][1].dot(mPoints[3]);

                k = mNormSquare[0][2] < mNormSquare[1][2]
                        ? (mNormSquare[0][2] < mNormSquare[3][2] ? 0 : 3)
                        : (mNormSquare[1][2] < mNormSquare[3][2] ? 1 : 3);
                mDet[0xf][2] = mDet[0xb][0] * mDiffLength[k][2].dot(mPoints[0])
                        + mDet[0xb][1] * mDiffLength[k][2].dot(mPoints[1])
                        + mDet[0xb][3] * mDiffLength[k][2].dot(mPoints[3]);

                k = mNormSquare[0][3] < mNormSquare[1][3]
                        ? (mNormSquare[0][3] < mNormSquare[2][3] ? 0 : 2)
                        : (mNormSquare[1][3] < mNormSquare[2][3] ? 1 : 2);
                mDet[0xf][3] = mDet[0x7][0] * mDiffLength[k][3].dot(mPoints[0])
                        + mDet[0x7][1] * mDiffLength[k][3].dot(mPoints[1])
                        + mDet[0x7][2] * mDiffLength[k][3].dot(mPoints[2]);
            }
        }
    }

    // Return true if the subset is a proper subset.
    // A proper subset X is a subset where for all point "y_i" in X, we have
    // detX_i value bigger than zero
    private boolean isProperSubset(int subset) {

        // For each four point of the possible simplex set
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(subset, bit) && mDet[subset][i] <= 0.0f) {
                return false;
            }
        }

        return true;
    }

    // Return true if the bits of "b" is a subset of the bits of "a"
    private boolean isSubset(int a, int b) {
        return ((a & b) == a);
    }

    // Return true if the subset is a valid one for the closest point computation.
    // A subset X is valid if :
    //    1. delta(X)_i > 0 for each "i" in I_x and
    //    2. delta(X U {y_j})_j <= 0 for each "j" not in I_x_
    private boolean isValidSubset(int subset) {

        // For each four point in the possible simplex set
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(mAllBits, bit)) {
                // If the current point is in the subset
                if (overlap(subset, bit)) {
                    // If one delta(X)_i is smaller or equal to zero
                    if (mDet[subset][i] <= 0.0f) {
                        // The subset is not valid
                        return false;
                    }
                } // If the point is not in the subset and the value delta(X U {y_j})_j
                // is bigger than zero
                else if (mDet[subset | bit][i] > 0.0f) {
                    // The subset is not valid
                    return false;
                }
            }
        }

        return true;
    }

    // Return true if some bits of "a" overlap with bits of "b"
    private boolean overlap(int a, int b) {
        return ((a & b) != 0x0);
    }

    // Update the cached values used during the GJK algorithm
    private void updateCache() {

        // For each of the four possible points of the simplex
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            // If the current points is in the simplex
            if (overlap(mBitsCurrentSimplex, bit)) {

                // Compute the distance between two points in the possible simplex set
                mDiffLength[i][mLastFound] = new Vector3(mPoints[i]).subtract(mPoints[mLastFound]);
                mDiffLength[mLastFound][i] = new Vector3(mDiffLength[i][mLastFound]).invert();

                // Compute the squared length of the vector
                // distances from points in the possible simplex set
                mNormSquare[i][mLastFound] = mNormSquare[mLastFound][i]
                        = mDiffLength[i][mLastFound].dot(mDiffLength[i][mLastFound]);
            }
        }
    }

    // Add a new support point of (A-B) into the simplex
    // suppPointA : support point of object A in a direction -v
    // suppPointB : support point of object B in a direction v
    // point      : support point of object (A-B) => point = suppPointA - suppPointB
    public void addPoint(Vector3 point, Vector3 suppPointA, Vector3 suppPointB) {

        assert (!isFull());

        mLastFound = 0;
        mLastFoundBit = 0x1;

        // Look for the bit corresponding to one of the four point that is not in
        // the current simplex
        while (overlap(mBitsCurrentSimplex, mLastFoundBit)) {
            mLastFound++;
            mLastFoundBit <<= 1;
        }

        assert (mLastFound >= 0 && mLastFound < 4);

        // Add the point into the simplex
        mPoints[mLastFound].set(point);
        mPointsLengthSquare[mLastFound] = point.dot(point);
        mAllBits = mBitsCurrentSimplex | mLastFoundBit;

        // Update the cached values
        updateCache();

        // Compute the cached determinant values
        computeDeterminants();

        // Add the support points of objects A and B
        mSuppPointsA[mLastFound].set(suppPointA);
        mSuppPointsB[mLastFound].set(suppPointB);
    }

    // Backup the closest point
    public void backupClosestPointInSimplex(Vector3 v) {

        float minDistSquare = Defaults.DECIMAL_LARGEST;

        for (int bit = mAllBits; bit != 0x0; bit--) {
            if (isSubset(bit, mAllBits) && isProperSubset(bit)) {
                Vector3 u = computeClosestPointForSubset(bit);
                float distSquare = u.dot(u);
                if (distSquare < minDistSquare) {
                    minDistSquare = distSquare;
                    mBitsCurrentSimplex = bit;
                    v.set(u);
                }
            }
        }
    }

    // Compute the closest point "v" to the origin of the current simplex.
    // This method executes the Jonhnson's algorithm for computing the point
    // "v" of simplex that is closest to the origin. The method returns true
    // if a closest point has been found.
    public boolean computeClosestPoint(Vector3 v) {

        // For each possible simplex set
        for (int subset = mBitsCurrentSimplex; subset != 0x0; subset--) {
            // If the simplex is a subset of the current simplex and is valid for the Johnson's
            // algorithm test
            if (isSubset(subset, mBitsCurrentSimplex) && isValidSubset(subset | mLastFoundBit)) {
                mBitsCurrentSimplex = subset | mLastFoundBit;                   // Add the last added point to the current simplex
                Vector3 u = computeClosestPointForSubset(mBitsCurrentSimplex);  // Compute the closest point in the simplex
                v.set(u);
                return true;
            }
        }

        // If the simplex that contains only the last added point is valid for the Johnson's algorithm test
        if (isValidSubset(mLastFoundBit)) {
            mBitsCurrentSimplex = mLastFoundBit;                // Set the current simplex to the set that contains only the last added point
            mMaxLengthSquare = mPointsLengthSquare[mLastFound]; // Update the maximum square length
            Vector3 u = mPoints[mLastFound];                    // The closest point of the simplex "v" is the last added point
            v.set(u);
            return true;
        }

        // The algorithm failed to found a point
        return false;
    }

    // Compute the closest points "pA" and "pB" of object A and B.
    // The points are computed as follows :
    //      pA = sum(lambda_i * a_i)    where "a_i" are the support points of object A
    //      pB = sum(lambda_i * b_i)    where "b_i" are the support points of object B
    //      with lambda_i = deltaX_i / deltaX
    public void computeClosestPointsOfAandB(Vector3 pA, Vector3 pB) {

        float deltaX = 0.0f;
        pA.zero();
        pB.zero();

        // For each four points in the possible simplex set
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            // If the current point is part of the simplex
            if (overlap(mBitsCurrentSimplex, bit)) {
                deltaX += mDet[mBitsCurrentSimplex][i];
                pA.add(new Vector3(mSuppPointsA[i]).multiply(mDet[mBitsCurrentSimplex][i]));
                pB.add(new Vector3(mSuppPointsB[i]).multiply(mDet[mBitsCurrentSimplex][i]));
            }
        }

        assert (deltaX > 0.0f);
        float factor = 1.0f / deltaX;
        pA.multiply(factor);
        pB.multiply(factor);
    }

    // Return the maximum squared length of a point
    public float getMaxLengthSquareOfAPoint() {
        return mMaxLengthSquare;
    }

    // Return the points of the simplex
    public int getSimplex(Vector3[] suppPointsA, Vector3[] suppPointsB, Vector3[] points) {
        int nbVertices = 0;

        // For each four point in the possible simplex
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {

            // If the current point is in the simplex
            if (overlap(mBitsCurrentSimplex, bit)) {

                // Store the points
                suppPointsA[nbVertices] = new Vector3(this.mSuppPointsA[nbVertices]);
                suppPointsB[nbVertices] = new Vector3(this.mSuppPointsB[nbVertices]);
                points[nbVertices] = new Vector3(this.mPoints[nbVertices]);

                nbVertices++;
            }
        }

        // Return the number of points in the simplex
        return nbVertices;
    }

    // Return true if the set is affinely dependent.
    // A set if affinely dependent if a point of the set
    // is an affine combination of other points in the set
    public boolean isAffinelyDependent() {

        float sum = 0.0f;

        // For each four point of the possible simplex set
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(mAllBits, bit)) {
                sum += mDet[mAllBits][i];
            }
        }

        return (sum <= 0.0f);
    }

    // Return true if the simple is empty
    public boolean isEmpty() {
        return (mBitsCurrentSimplex == 0x0);
    }

    // Return true if the simplex contains 4 points
    public boolean isFull() {
        return (mBitsCurrentSimplex == 0xf);
    }

    // Return true if the point is in the simplex
    public boolean isPointInSimplex(Vector3 point) {

        // For each four possible points in the simplex
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            // Check if the current point is in the simplex
            if (overlap(mAllBits, bit) && point.equals(mPoints[i])) {
                return true;
            }
        }

        return false;
    }

}
