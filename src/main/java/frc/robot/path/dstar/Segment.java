package frc.robot.path.dstar;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;;

public interface Segment {
  public Point getStart();
  public Point getEnd();
  public double getLength();

  // differential drive
  public Rotation2d getRotation(double distance);
  public Rotation2d angularVelocity(double distance, double speed);
  public Rotation2d angularVelocity(double distance);

  // holonomic drive
  public Translation2d getVelocity(double distance, double speed);
  public Translation2d getVelocity(double distance);
  public Translation2d getAcceleration(double distance, double speed, double accel);
  public Translation2d getAcceleration(double distance, double speed);
  public Translation2d getAcceleration(double distance);

  // transformations
  public Segment translate(Translation2d offset);
  public Segment rotate(Rotation2d rotation);
  public Segment rotate(Transform2d transformation);
}