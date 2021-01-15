package frc.robot.mapping;

import java.util.function.BiFunction;
import java.util.function.UnaryOperator;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * A path consisting of other paths in sequence.
 */
public class CompoundPath implements Path {
    private final Path[] segments;

    /**
     * Creates a new CompoundPath from a seriers of path segments.
     * 
     * @param segments  The path segments to concatentate into the new path.
     */
    public CompoundPath(Path... segments) {
        this.segments = segments;
    }

    @Override
    public double getLength() {
        double length = 0;
        for(int i=0; i<segments.length; i++)
            length += segments[i].getLength();
        return length;
    }

    /**
     * Evaluates a function at a specific distance along this path.
     * 
     * @param func  The function to evaluate. This function must take the path
     *  segment at the specified distance and a distance along that path and
     *  return a value.
     * @param distance  The distance along this path to evaluate the function.
     */
    protected <T> T evalAtDistance(BiFunction<Path, Double, T> func, double distance) {
        for(int i=0; i<segments.length; i++){
            if(distance > segments[i].getLength())
                distance -= segments[i].getLength();
            else
                return func.apply(segments[i], distance);
        }
        return null;
    }

    @Override
    public Point getPos(double distance) {
        return evalAtDistance((seg, dist) -> seg.getPos(dist), distance);
    }

    @Override
    public Point getEnd() {
        return segments[segments.length-1].getEnd();
    }

    @Override
    public Rotation2d getRotation(double distance) {
        return evalAtDistance((seg, dist) -> seg.getRotation(dist), distance);
    }

    @Override
    public Rotation2d getAngularVelocity(double distance) {
        return evalAtDistance((seg, dist) -> seg.getAngularVelocity(dist), distance);
    }

    /**
     * Applies a transformation to all segments of this path.
     * 
     * @param transformation  The transformation to apply. This function must
     *  take one path segment as input and return a modified path segment.
     * @return A new CompoundPath consisting of the images of the path segments
     *  of this path after the transformation.
     */
    protected CompoundPath applyToAll(UnaryOperator<Path> transformation) {
        Path[] newSegments = new Path[segments.length];
        for(int i=0; i<segments.length; i++)
            newSegments[i] = transformation.apply(segments[i]);
        return new CompoundPath(newSegments);
    }

    @Override
    public CompoundPath translateBy(Translation2d offset) {
        return applyToAll(segment -> segment.translateBy(offset));
    }

    @Override
    public CompoundPath rotateBy(Rotation2d rotation) {
        return applyToAll(segment -> segment.rotateBy(rotation));
    }

    @Override
    public CompoundPath reverse() {
        Path[] newSegments = new Path[segments.length];
        for(int i=segments.length-1; i>=0; i--)
            newSegments[i] = segments[i].reverse();
        return new CompoundPath(newSegments);
    }

    @Override
    public double getDistance(Point from) {
        double minDist = Double.POSITIVE_INFINITY;
        for(int i=0; i<segments.length; i++)
            minDist = Math.min(minDist, segments[i].getDistance(from));
        return minDist;
    }

    @Override
    public double getDistance(Path other) {
        double minDist = Double.POSITIVE_INFINITY;
        for(int i=0; i<segments.length; i++)
            minDist = Math.min(minDist, segments[i].getDistance(other));
        return minDist;
    }
}