// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class DifferentialDriveOdometryCommand extends CommandBase {

  private final DifferentialDriveSubsystem driveSub;
  private final NavigationSubsystem navSub;
  private final DifferentialDriveOdometry odometry;

  /** Creates a new OdometryCommand. */
  public DifferentialDriveOdometryCommand(DifferentialDriveSubsystem driveSub, NavigationSubsystem navSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveSub, navSub);

    this.driveSub = driveSub;
    this.navSub = navSub;
    odometry = new DifferentialDriveOdometry(navSub.getHeadingRotation());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("Odometry command initialized.");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    updateOdometry();

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
  public void updateOdometry() {

    odometry.update(navSub.getHeadingRotation(), driveSub.getEncoderDist(true), driveSub.getEncoderDist(false));

  }

  // simulation methods

}
