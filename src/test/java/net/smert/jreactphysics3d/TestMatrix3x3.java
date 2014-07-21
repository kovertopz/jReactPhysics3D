package net.smert.jreactphysics3d;

import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestMatrix3x3 {

    @Test
    public void testClassExists() {
        Assert.assertThat(new Matrix3x3(), CoreMatchers.instanceOf(Matrix3x3.class));
    }

}
