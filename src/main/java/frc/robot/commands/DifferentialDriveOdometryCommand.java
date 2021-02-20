package frc.robot.commands;

import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * 
 */
public class DifferentialDriveOdometryCommand extends CommandBase {
  // ALL ODOMETRY IS DONE IN METERS, NOT IMPERIAL!!!
  private final DifferentialDriveSubsystem driveSub;
  private final NavigationSubsystem navSub;
  private final DifferentialDriveOdometry odometry;

  private FieldDisplayCommand fieldDisplayCommand;
  private boolean useFieldDisplay;

  /**
   * Creates a new OdometryCommand without a field simulation.
   * @param driveSub - Differential drive subsystem.
   * @param navSub   - Navigation subsystem.
   */
  public DifferentialDriveOdometryCommand(DifferentialDriveSubsystem driveSub, NavigationSubsystem navSub) {
    this.driveSub = driveSub;
    this.navSub = navSub;
    odometry = new DifferentialDriveOdometry(navSub.getHeadingRotation2d());
    useFieldDisplay = false;
  }

  /**
   * Creates a new OdometryCommand with a field simulation.
   * @param driveSub - Differential drive subsystem.
   * @param navSub   - Navigation subsystem.
   * @param fieldDisplay - Field display command.
   */
  public DifferentialDriveOdometryCommand(DifferentialDriveSubsystem driveSub, NavigationSubsystem navSub, FieldDisplayCommand fieldDisplayCommand) {
    this(driveSub, navSub);    
    this.fieldDisplayCommand = fieldDisplayCommand;
    useFieldDisplay = true;
  }

  @Override
  public void initialize() {
    System.out.println("Differential drive odometry command initialized.");
  }

  @Override
  public void execute() {
    updateOdometry();

    if (useFieldDisplay) updateFieldSim();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Odometry Methods

  /**
   * Gets current DifferentialDriveOdometry.
   * @return Differential drive odometry.
   */
  public DifferentialDriveOdometry getOdometryCopy() {
    return new DifferentialDriveOdometry(getPose().getRotation(), getPose()); // TODO: Return a TRUE deep copy of the object
  }

  /**
   * Updates differential drive odometry.
   */
  private void updateOdometry() {
    odometry.update(navSub.getHeadingRotation2d(), driveSub.getEncoderDist(true), driveSub.getEncoderDist(false));
  }

  /**
   * Resets differential drive odometry.
   */
  public void resetOdometry(Pose2d poseMeters, Rotation2d gyroAngle) {
    odometry.resetPosition(poseMeters, gyroAngle);
    driveSub.resetEncoderCounts();
  }

  /**
   * Gets odometry x measurement (meters).
   * @return X (meters).
   */
  public double getX() {
    return getPose().getX();
  }

  /**
   * Gets odometry y measurement (meters).
   * @return Y (meters).
   */
  public double getY() {
    return getPose().getY();
  }

  /**
   * Gets odometry theta measurement (degrees).
   * @return Theta (degrees).
   */
  public double getTheta() {
    return getPose().getRotation().getDegrees();
  }

  /**
   * Gets the pose in meters
   * @return Pose2d object with x, y, and theta
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Gets filterable vector motion measurement from odometry.
   * @return Odometry vector measurement (x, y, theta).
   */
  public SimpleMatrix getOdometryVector() {
    return new SimpleMatrix(new double[][] { { getX() }, { getY() }, { getTheta() }});
  }

  // simulation methods

  /**
   * Updates field simulation
   */
  private void updateFieldSim() {
    fieldDisplayCommand.setRobotPose(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), navSub.getHeadingRotation2d());
  }
}
