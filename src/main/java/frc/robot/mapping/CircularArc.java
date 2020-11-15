package frc.robot.mapping;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class CircularArc implements Path {
  protected final Point center;
  protected final double theta;
  protected final double radius;
  protected final Point off;

  public CircularArc(Point center, Point start, double theta) {
    this.center = center;
    this.theta = theta;
    radius = center.getDistance(start);
    off = start.minus(start);
  }
  public CircularArc(Point start, Point center, Point end) {
    if(center.getDistance(start) != center.getDistance(end))
      throw new IllegalArgumentException("start and end must be equidistant from center.");
    this.center = center;
    theta = Point.getAngle(start, center, end).getRadians();
    radius = center.getDistance(start);
    off = start.minus(start);
  }
  protected CircularArc(Point center, double theta, Point off) {
    this.center = center;
    this.theta = theta;
    this.off = off;
    this.radius = off.getNorm();
  }

  public double getLength() {
    return radius * theta;
  }

  public Point getPos(double distance) {
    return off.rotateBy(new Rotation2d(distance / radius)).plus(center);
  }

  public Rotation2d getRotation(double distance) {
    double phi = Math.copySign(Math.PI / 2 + distance / radius, theta);
    return new Rotation2d(phi).plus(off.getAngle());
  }

  public Rotation2d getAngularVelocity(double distance) {
    return new Rotation2d(Math.signum(theta) / radius);
  }

  public CircularArc translateBy(Translation2d offset) {
    return new CircularArc(center.plus(offset), getStart().plus(offset), theta);
  }

  public CircularArc rotateBy(Rotation2d rotation) {
    return new CircularArc(center.rotateBy(rotation), getStart().rotateBy(rotation), theta);
  }

  protected boolean includesRay(Point ray) {
    if(theta > 2*Math.PI || theta < 2*Math.PI) return true;
    Rotation2d phi = Point.getAngle(off, center, ray.minus(center));
    Rotation2d acuteTheta = new Rotation2d(theta);
    if(phi.equals(acuteTheta)) return true;
    return (phi.getCos() > acuteTheta.getCos() &&
        phi.getSin() * acuteTheta.getSin() > 0) ^
        acuteTheta.getRadians() == theta;
  }

  public double getDistance(Point from) {
    if(includesRay(from)) return Math.abs(center.getDistance(from) - radius);
    else return Math.min(getStart().getDistance(from), getEnd().getDistance(from));
  }

  public CircularArc reverse() {
    return new CircularArc(center, -theta, off.rotateBy(new Rotation2d(theta)));
  }

  public double getDistance(LinearSegment line) {
    Point nearest = line.getNearest(center);
    double minDist = center.getDistance(nearest);
    if(minDist >= radius){
      if(includesRay(nearest)) return minDist - radius;
      return Math.min(line.getDistance(getStart()), line.getDistance(getEnd()));
    }
    Point start = line.getStart(), end = line.getEnd();
    Point farthest = center.getDistance(start) > center.getDistance(end) ? start : end;
    double maxDist = center.getDistance(farthest);
    if(maxDist <= radius){
      if(includesRay(farthest)) return radius - maxDist;
      return Math.min(line.getDistance(getStart()), line.getDistance(getEnd()));
    }
    Rotation2d phi = new Rotation2d(Math.acos(minDist / radius));
    Point a = new Point(radius, nearest.getAngle().plus(phi)).plus(center);
    Point b = new Point(radius, nearest.getAngle().minus(phi)).plus(center);
    if(Point.getAngle(start, a, end).getCos() == -1 && includesRay(a))
      return 0;
    if(Point.getAngle(start, b, end).getCos() == -1 && includesRay(b))
      return 0;
    return Math.min(line.getDistance(getStart()), line.getDistance(getEnd()));
  }
  public double getDistance(CircularArc other) {
    double dist = center.getDistance(other.center);
    if(dist >= radius + other.radius && includesRay(other.center) && other.includesRay(center))
      return dist - radius - other.radius;
    if(dist < radius + other.radius){
      Rotation2d phi = new Rotation2d(Math.acos(radius*radius/dist));
      Rotation2d angle = other.center.minus(center).getAngle();
      Point a = center.plus(new Point(radius, angle.plus(phi)));
      if(includesRay(a) && other.includesRay(a)) return 0;
      a = center.plus(new Point(radius, angle.minus(phi)));
      if(includesRay(a) && other.includesRay(a)) return 0;
    }
    return Math.min(Math.min(other.getDistance(getStart()), other.getDistance(getEnd())),
        Math.min(getDistance(other.getStart()), getDistance(other.getEnd())));
  }
}