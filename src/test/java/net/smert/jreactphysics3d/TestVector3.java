package net.smert.jreactphysics3d;

import net.smert.jreactphysics3d.mathematics.Vector3;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestVector3 {

    @Test
    public void testClassExists() {
        Assert.assertThat(new Vector3(), CoreMatchers.instanceOf(Vector3.class));
    }

}
