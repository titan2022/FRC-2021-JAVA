// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.vision.LimelightMath;
import frc.robot.config.XboxMap;

public class LimelightCommand extends CommandBase {
  private static LimelightSubsystem limelightSubsystem;
  private static LimelightMath limelightMath;
  private static DifferentialDriveSubsystem difDriveSubsystem;
  private double Kp;
  private double desired_distance=1.0f;
  public LimelightCommand() {
    addRequirements(limelightSubsystem);
    addRequirements(difDriveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  
    if(XboxMap.pressA())
    {
      if(limelightSubsystem.validTarget()!=true)
      {
        difDriveSubsystem.setOutput(ControlMode.PercentOutput, 0.3f, -0.3f );
      }
    }
    
    if(XboxMap.releaseA())
    {
      double adjust = Kp *(desired_distance-limelightMath.calculateDistance());
      difDriveSubsystem.setOutput(ControlMode.PercentOutput, adjust, adjust );
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }
}
