package frc.robot.path.dstar;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class LinearSegment implements Segment {
  public final Point start;
  public final Point end;
  
  public LinearSegment(Point start, Point end) {
    this.start = start;
    this.end = end;
  }

  public Point getStart() {
    return start;
  }

  public Point getEnd() {
    return end;
  }

  public double getLength() {
    return start.distance(end);
  }

  public Rotation2d getRotation(double distance) {
    return new Rotation2d(end.x - start.x, end.y - start.y);
  }

  public Rotation2d angularVelocity(double distance) {
    return new Rotation2d();
  }

  public LinearSegment translate(Translation2d offset) {
    return new LinearSegment(
        Point.fromTranslation(start.translation().plus(offset)),
        Point.fromTranslation(end.translation().plus(offset)));
  }
  
  public LinearSegment rotate(Rotation2d rotation) {
    return new LinearSegment(
        Point.fromTranslation(start.translation().rotateBy(rotation)), 
        Point.fromTranslation(end.translation().rotateBy(rotation)));
  }
}