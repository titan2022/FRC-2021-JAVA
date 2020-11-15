package frc.robot.mapping;

import java.util.Comparator;
import java.util.Iterator;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public interface Obstacle extends Comparator<Point> {
    default public boolean isClear(Path path, double radius) {
        throw new UnsupportedOperationException();
    }
    default public boolean isClear(Path path) {
        return isClear(path, 0);
    }

    public Iterable<Point> getEndpoints(Point source, double radius);
    default public Iterable<Point> getEndpoints(Point source) {
        return getEndpoints(source, 0);
    }

    default public Iterable<LinearSegment> getTangents(Obstacle other, double radius) {
        try{
            Obstacle self = this;
            return new Iterable<LinearSegment>(){
                private Iterable<LinearSegment> original = other.getTangents(self, radius);

                @Override
                public Iterator<LinearSegment> iterator() {
                    return new Iterator<LinearSegment>(){
                        private Iterator<LinearSegment> it = original.iterator();

                        @Override
                        public boolean hasNext() {
                            return it.hasNext();
                        }

                        @Override
                        public LinearSegment next() {
                            return it.next().reverse();
                        }
                    };
                }
            };
        }
        catch(StackOverflowError err){
            throw new UnsupportedOperationException(err);
        }
    }
    default public Iterable<LinearSegment> getTangents(Obstacle other) {
        return getTangents(other, 0);
    }

    public int compare(Point a, Point b);

    public Obstacle translateBy(Translation2d offset);

    public Obstacle rotateBy(Rotation2d rotation);

    default public boolean isEndpoint(Point endpoint, Point other, double radius) {
        for(Point canidate : getEndpoints(other, radius))
            if(canidate.equals(endpoint)) return true;
        return false;
    }
    default public boolean isEndpoint(Point endpoint, Point other) {
        return isEndpoint(endpoint, other, 0);
    }
}