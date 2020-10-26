package frc.robot.path.dstar;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class CircularArc implements Segment {
  private final Translation2d center;
  private final Translation2d start;
  private final Rotation2d theta;

  public CircularArc(Point center, Point start, Point end) {
    this.center = center.translation();
    this.start = start.translation().minus(this.center);
    theta = Rotation2d.fromDegrees(Point.getAngle(start, center, end));
  }
  public CircularArc(Translation2d center, Translation2d start, Rotation2d theta) {
    this.center = center;
    this.start = start;
    this.theta = theta;
  }

  public Point getStart() {
    return Point.fromTranslation(start);
  }

  public Point getEnd() {
    return Point.fromTranslation(start.rotateBy(theta).plus(center));
  }

  public double getLength() {
    return start.getNorm() * theta.getRadians();
  }

  public Rotation2d getRotation(double distance) {
    return new Rotation2d(-start.getY(), start.getX())
        .plus(new Rotation2d(distance / start.getNorm()));
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