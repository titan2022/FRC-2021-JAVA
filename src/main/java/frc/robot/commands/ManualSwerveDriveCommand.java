// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;
import frc.robot.motion.control.PIDConfig;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

/**
 * 
 */
public class ManualSwerveDriveCommand extends CommandBase {
  private static SwerveDriveSubsystem swerveDriveSubsystem;
  private static NavigationSubsystem navigationSubsystem;
  private PIDController pid;
  private boolean brakeState = false;

  /** Creates a new ManualDifferentialDriveCommand. */
  public ManualSwerveDriveCommand(SwerveDriveSubsystem swerveDriveSub, NavigationSubsystem navSub, PIDConfig pidConfig) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSub);
    swerveDriveSubsystem = swerveDriveSub;
    navigationSubsystem = navSub;

    pid = new PIDController(pidConfig.kP, pidConfig.kI, pidConfig.kD);
    pid.enableContinuousInput(0, 2*Math.PI);
    pid.setIntegratorRange(pidConfig.INTEGRATION_MIN, pidConfig.INTEGRATION_MAX);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Manual Swerve Drive Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    brakeState = XboxMap.toggleBrakes() ? !brakeState : brakeState;

    if (brakeState) {
      swerveDriveSubsystem.enableBrakes();
    }
    else { 
      swerveDriveSubsystem.disableBrakes();
      swerveDriveSubsystem.setOutput(new ChassisSpeeds(XboxMap.leftX(), XboxMap.leftY(), pid.calculate(navigationSubsystem.getHeadingRadians(), Math.atan2(XboxMap.rightY(), XboxMap.rightX()))));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stop();
		System.out.println("Manual Swerve Drive Command Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
