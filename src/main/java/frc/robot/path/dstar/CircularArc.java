package frc.robot.path.dstar;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class CircularArc implements Segment {
  private final Point center;
  private final Point start;
  private final Rotation2d theta;

  public CircularArc(Point center, Point start, Point end) {
    this.center = center;
    this.start = start.minus(this.center);
    theta = Point.getAngle(start, center, end);
  }
  public CircularArc(Point center, Translation2d start, Rotation2d theta) {
    this.center = center;
    this.start = new Point(start);
    this.theta = theta;
  }

  public Point getStart() {
    return center.plus(start);
  }

  public Point getEnd() {
    return center.plus(start.rotateBy(theta));
  }

  public double getLength() {
    return start.getNorm() * theta.getRadians();
  }

  public Point getPos(double distance) {
    return start
        .rotateBy(new Rotation2d(distance / start.getNorm())
            .times(Math.signum(theta.getRadians())))
        .plus(center);
  }

  public Rotation2d getRotation(double distance) {
    return new Rotation2d(distance / start.getNorm() + Math.PI)
        .times(Math.signum(theta.getRadians())).plus(start.angle());
  }

  public Rotation2d angularVelocity(double distance) {
    return new Rotation2d(1/start.getNorm());
  }

  public CircularArc translate(Translation2d offset) {
    return new CircularArc(center.plus(offset), start, theta);
  }

  public CircularArc rotate(Rotation2d rotation) {
    return new CircularArc(center.rotateBy(rotation), start.rotateBy(rotation), theta);
  }
}