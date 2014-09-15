package net.smert.jreactphysics3d.collision;

/**
 * typedef std::pair<bodyindex, bodyindex> bodyindexpair;
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BodyIndexPair {

    private final int first;
    private final int second;

    public BodyIndexPair(int first, int second) {
        this.first = first;
        this.second = second;
    }

    public int getFirst() {
        return first;
    }

    public int getSecond() {
        return second;
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 29 * hash + this.first;
        hash = 29 * hash + this.second;
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
        final BodyIndexPair other = (BodyIndexPair) obj;
        if (this.first != other.first) {
            return false;
        }
        return this.second == other.second;
    }

}
