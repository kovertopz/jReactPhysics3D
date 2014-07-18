package net.smert.jreactphysics3d.body;

/**
 * typedef std::pair<bodyindex, bodyindex> bodyindexpair;
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BodyIndexPair {

    public int first;
    public int second;

    public BodyIndexPair(int first, int second) {
        this.first = first;
        this.second = second;
    }

}
