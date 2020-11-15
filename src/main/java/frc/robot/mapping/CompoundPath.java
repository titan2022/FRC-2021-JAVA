package frc.robot.mapping;

import java.util.function.BiFunction;
import java.util.function.UnaryOperator;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class CompoundPath implements Path {
    private final Path[] segments;

    public CompoundPath(Path... segments) {
        this.segments = segments;
    }

    public double getLength() {
        double length = 0;
        for(int i=0; i<segments.length; i++)
            length += segments[i].getLength();
        return length;
    }

    protected <T> T evalAtDistance(BiFunction<Path, Double, T> func, double distance) {
        for(int i=0; i<segments.length; i++){
            if(distance > segments[i].getLength())
                distance -= segments[i].getLength();
            else
                return func.apply(segments[i], distance);
        }
        return null;
    }

    public Point getPos(double distance) {
        return evalAtDistance((seg, dist) -> seg.getPos(dist), distance);
    }

    public Point getEnd() {
        return segments[segments.length-1].getEnd();
    }

    public Rotation2d getRotation(double distance) {
        return evalAtDistance((seg, dist) -> seg.getRotation(dist), distance);
    }

    public Rotation2d getAngularVelocity(double distance) {
        return evalAtDistance((seg, dist) -> seg.getAngularVelocity(dist), distance);
    }

    protected CompoundPath applyToAll(UnaryOperator<Path> transformation) {
        Path[] newSegments = new Path[segments.length];
        for(int i=0; i<segments.length; i++)
            newSegments[i] = transformation.apply(segments[i]);
        return new CompoundPath(newSegments);
    }

    public CompoundPath translateBy(Translation2d offset) {
        return applyToAll(segment -> segment.translateBy(offset));
    }

    public CompoundPath rotateBy(Rotation2d rotation) {
        return applyToAll(segment -> segment.rotateBy(rotation));
    }

    public CompoundPath reverse() {
        Path[] newSegments = new Path[segments.length];
        for(int i=segments.length-1; i>=0; i--)
            newSegments[i] = segments[i].reverse();
        return new CompoundPath(newSegments);
    }

    public double getDistance(Point from) {
        double minDist = Double.POSITIVE_INFINITY;
        for(int i=0; i<segments.length; i++)
            minDist = Math.min(minDist, segments[i].getDistance(from));
        return minDist;
    }

    public double getDistance(Path other) {
        double minDist = Double.POSITIVE_INFINITY;
        for(int i=0; i<segments.length; i++)
            minDist = Math.min(minDist, segments[i].getDistance(other));
        return minDist;
    }
}