package net.smert.jreactphysics3d;

import net.smert.jreactphysics3d.mathematics.Quaternion;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestQuaternion {

    @Test
    public void testClassExists() {
        Assert.assertThat(new Quaternion(), CoreMatchers.instanceOf(Quaternion.class));
    }

}
