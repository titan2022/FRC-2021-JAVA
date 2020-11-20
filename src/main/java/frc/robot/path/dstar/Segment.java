package frc.robot.path.dstar;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import frc.robot.mapping.Point;

public interface Segment {
  public Point getStart();
  public Point getEnd();
  public double getLength();
  public Point getPos(double distance);

  // differential drive
  public Rotation2d getRotation(double distance);
  public default Rotation2d angularVelocity(double distance, double speed) {
    return angularVelocity(distance).times(speed);
  }
  public Rotation2d angularVelocity(double distance);

  // holonomic drive
  public default Translation2d getVelocity(double distance, double speed) {
    return getVelocity(distance).times(speed);
  }
  public default Translation2d getVelocity(double distance) {
    return new Translation2d(1, 0).rotateBy(getRotation(distance));
  }
  public default Translation2d getAcceleration(double distance, double speed, double accel) {
    return getAcceleration(distance, speed)
      .plus(new Translation2d(accel, 0).rotateBy(getRotation(distance)));
  }
  public default Translation2d getAcceleration(double distance, double speed) {
    return getAcceleration(distance).times(speed);
  }
  public default Translation2d getAcceleration(double distance) {
    return new Translation2d(angularVelocity(distance).getRadians(), 0)
      .rotateBy(getRotation(distance));
  }

  // transformations
  public Segment translate(Translation2d offset);
  public Segment rotate(Rotation2d rotation);
  public default Segment rotate(Rotation2d rotation, Point center){
    return translate(center.unaryMinus()).rotate(rotation).translate(center);
  }
}