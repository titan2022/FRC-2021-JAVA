package frc.robot.mapping;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class LinearSegment implements Path {
    protected final Point start;
    protected final Point end;

    public LinearSegment(Point start, Point end) {
        this.start = start;
        this.end = end;
    }

    public double getLength() {
        return end.minus(start).getNorm();
    }

    public Point getPos(double distance) {
        return start.plus(new Point(distance, end.minus(start).getAngle()));
    }

    public Rotation2d getRotation(double distance) {
        return end.minus(start).getAngle();
    }

    public Rotation2d getAngularVelocity(double distance) {
        return new Rotation2d(0);
    }

    public LinearSegment translateBy(Translation2d offset) {
        return new LinearSegment(start.plus(offset), end.plus(offset));
    }

    public LinearSegment rotateBy(Rotation2d rotation) {
        return new LinearSegment(start.rotateBy(rotation), end.rotateBy(rotation));
    }

    public double getDistance(Point from) {
        Rotation2d theta = Point.getAngle(end, start, from);
        double startDist = start.getDistance(from);
        if(theta.getCos() >= 0 && theta.getCos() * startDist < getLength())
            return theta.getSin() * startDist;
        return Math.min(startDist, end.getDistance(from));
    }

    public Point getNearest(Point from) {
        Rotation2d theta = Point.getAngle(end, start, from);
        if(theta.getCos() <= 0) return start;
        double r = theta.getCos() * start.getDistance(from);
        if(r >= getLength()) return end;
        return start.plus(new Point(r, end.minus(start).getAngle()));
    }

    public boolean intersects(LinearSegment other) {
        double abx, aby, xya, xyb;
        abx = Point.getAngle(start, end, other.start).getCos();
        aby = Point.getAngle(start, end, other.end).getCos();
        xya = Point.getAngle(other.start, other.end, start).getCos();
        xyb = Point.getAngle(other.start, other.end, end).getCos();
        if(abx * aby <= 0 && xya * xyb <= 0) return true;
        else return false;
    }

    public double getDistance(LinearSegment other) {
        if(intersects(other)) return 0;
        else return Math.min(getDistance(other.start), getDistance(other.end));
    }

    public LinearSegment reverse() {
        return new LinearSegment(end, start);
    }
}