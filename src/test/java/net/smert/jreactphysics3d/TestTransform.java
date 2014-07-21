package net.smert.jreactphysics3d;

import net.smert.jreactphysics3d.mathematics.Transform;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestTransform {

    @Test
    public void testClassExists() {
        Assert.assertThat(new Transform(), CoreMatchers.instanceOf(Transform.class));
    }

}
