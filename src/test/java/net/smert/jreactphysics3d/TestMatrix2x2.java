package net.smert.jreactphysics3d;

import net.smert.jreactphysics3d.mathematics.Matrix2x2;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestMatrix2x2 {

    /// Identity transform
    private Matrix2x2 mIdentity;

    /// First example matrix
    private Matrix2x2 mMatrix1;

    @Before
    public void beforeEachTest() {
        mIdentity = Matrix2x2.identity();
        mMatrix1 = new Matrix2x2(2.0f, 24.0f, -4.0f, 5.0f);
    }

    @Test
    public void testClassExists() {
        Assert.assertThat(new Matrix2x2(), CoreMatchers.instanceOf(Matrix2x2.class));
    }

    @Test
    public void testConstructors() {

        Matrix2x2 test1 = new Matrix2x2(5.0f);
        Matrix2x2 test2 = new Matrix2x2(2.0f, 3.0f, 4.0f, 5.0f);
        Matrix2x2 test3 = new Matrix2x2(mMatrix1);

        Assert.assertEquals(test1.m[0][0], 5.0f, 0.0f);
        Assert.assertEquals(test1.m[0][1], 5.0f, 0.0f);
        Assert.assertEquals(test1.m[1][0], 5.0f, 0.0f);
        Assert.assertEquals(test1.m[1][1], 5.0f, 0.0f);

        Assert.assertEquals(test2.m[0][0], 2.0f, 0.0f);
        Assert.assertEquals(test2.m[0][1], 3.0f, 0.0f);
        Assert.assertEquals(test2.m[1][0], 4.0f, 0.0f);
        Assert.assertEquals(test2.m[1][1], 5.0f, 0.0f);

        Assert.assertEquals(test3.equals(mMatrix1), true);
    }

    @Test
    public void testGetSet() {
    }

    @Test
    public void testIdentity() {
    }

    @Test
    public void testOthersMethods() {
    }

    @Test
    public void testOperators() {
    }

}
