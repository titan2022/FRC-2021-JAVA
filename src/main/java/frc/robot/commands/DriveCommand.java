/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.XboxMap;

/**
 * An example command that uses an example subsystem.
 */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveSubsystem driveSubsystem;
  private boolean brakeState = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(SwerveDriveSubsystem subsystem) {
    driveSubsystem = new SwerveDriveSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("starting drive command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (XboxMap.toggleBrakes()) { brakeState = !brakeState; } // TODO: Change to ternary operator

    if (brakeState) {
      driveSubsystem.enableBrakes();
      driveSubsystem.enableRotatorBrakes();
    }
    else { 
      driveSubsystem.disableBrakes();
      driveSubsystem.setOutput(XboxMap.right(), XboxMap.leftX(), XboxMap.leftY());
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveSubsystem.stop();
      System.out.println("Swerve done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
