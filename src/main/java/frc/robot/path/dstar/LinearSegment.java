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
    return start.getDistance(end);
  }

  public Point getPos(double distance) {
    return new Point(distance, getRotation(0)).plus(start);
  }

  public Rotation2d getRotation(double distance) {
    return end.minus(start).angle();
  }

  public Rotation2d angularVelocity(double distance) {
    return new Rotation2d();
  }

  public LinearSegment translate(Translation2d offset) {
    return new LinearSegment(start.plus(offset), end.plus(offset));
  }
  
  public LinearSegment rotate(Rotation2d rotation) {
    return new LinearSegment(start.rotateBy(rotation), end.rotateBy(rotation));
  }
}