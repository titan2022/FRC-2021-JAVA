// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.XboxMap;
import frc.robot.subsystems.DriveSubsystem;

/**
 * 
 */
public class ManualDifferentialDriveCommand extends CommandBase {
  private static DriveSubsystem driveSubsystem;
  private boolean brakeState = false;

  /** Creates a new ManualDifferentialDriveCommand. */
  public ManualDifferentialDriveCommand(DriveSubsystem differentialDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(differentialDrive);
    driveSubsystem = differentialDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Manual Differential Drive Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (XboxMap.toggleBrakes()) { brakeState = !brakeState; } // TODO: Change to ternary operator

    if (brakeState) {
      driveSubsystem.enableBrakes();
    }
    else { 
      driveSubsystem.disableBrakes();
      driveSubsystem.setOutput(ControlMode.PercentOutput, XboxMap.left(), XboxMap.right());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
		System.out.println("Manual Differential Drive Command Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
