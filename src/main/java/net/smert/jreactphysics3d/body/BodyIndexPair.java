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
        if (this.second != other.second) {
            return false;
        }
        return true;
    }

}
