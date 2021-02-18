// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DifferentialDriveOdometryCommand extends CommandBase {

  // ALL ODOMETRY DONE IN METERS, not imperial

  private final DifferentialDriveSubsystem driveSub;
  private final NavigationSubsystem navSub;
  private final DifferentialDriveOdometry odometry;
  private boolean useFieldSim;
  private Field2d fieldSim;

  /**
   * Creates a new OdometryCommand without a field simulation.
   * 
   * @param driveSub - Differential drive subsystem.
   * @param navSub   - Navigation subsystem.
   */
  public DifferentialDriveOdometryCommand(DifferentialDriveSubsystem driveSub, NavigationSubsystem navSub) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSub = driveSub;
    this.navSub = navSub;
    odometry = new DifferentialDriveOdometry(navSub.getHeadingRotation());
    useFieldSim = false;

  }

  /**
   * Creates a new OdometryCommand with a field simulation.
   * 
   * @param driveSub - Differential drive subsystem.
   * @param navSub   - Navigation subsystem.
   * @param fieldSim - Field2d to be simulated.
   */
  public DifferentialDriveOdometryCommand(DifferentialDriveSubsystem driveSub, NavigationSubsystem navSub, Field2d fieldSim) {

    this(driveSub, navSub);
    this.fieldSim = fieldSim;
    useFieldSim = true;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("Differential drive odometry command initialized.");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    updateOdometry();

    if (useFieldSim) updateFieldSim();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

  // odometry methods

  /**
   * Gets current DifferentialDriveOdometry.
   * 
   * @return Differential drive odometry.
   */
  public DifferentialDriveOdometry getOdometry() {

    return odometry;

  }

  /**
   * Updates differential drive odometry.
   */
  private void updateOdometry() {

    odometry.update(navSub.getHeadingRotation(), driveSub.getEncoderDist(true), driveSub.getEncoderDist(false));

  }

  /**
   * Resets differential drive odometry.
   */
  public void resetOdometry() {

    odometry.resetPosition(new Pose2d(getX(), getY(), navSub.getHeadingRotation()), navSub.getHeadingRotation());

  }

  /**
   * Gets odometry x measurement (meters).
   * @return X (meters).
   */
  public double getX() {

    return odometry.getPoseMeters().getX();

  }

  /**
   * Gets odometry y measurement (meters).
   * @return Y (meters).
   */
  public double getY() {

    return odometry.getPoseMeters().getY();

  }

  /**
   * Gets filterable vector motion measurement from odometry.
   * 
   * @return Odometry vector measurement.
   */
  public SimpleMatrix getOdometryVector() {

    return new SimpleMatrix(new double[][] { { getX() }, { 0 }, { 0 }, { getY() }, { 0 }, { 0 } });

  }

  // simulation methods

  /**
   * Updates field simulation
   */
  private void updateFieldSim() {

    fieldSim.setRobotPose(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), navSub.getHeadingRotation());
    SmartDashboard.putData("Field", fieldSim);

  }

}
