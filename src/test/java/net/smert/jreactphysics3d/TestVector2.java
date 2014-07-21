package net.smert.jreactphysics3d;

import net.smert.jreactphysics3d.mathematics.Vector2;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TestVector2 {

    @Test
    public void testClassExists() {
        Assert.assertThat(new Vector2(), CoreMatchers.instanceOf(Vector2.class));
    }

}
