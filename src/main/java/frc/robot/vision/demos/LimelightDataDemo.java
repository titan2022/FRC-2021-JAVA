/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision.demos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.LimelightMath;
import frc.robot.vision.LimelightEnum;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightDataDemo extends CommandBase {
  /**
   * Creates a new LimelightDataDemo.
   */
  LimelightSubsystem subsystem = new LimelightSubsystem();
  LimelightMath math = new LimelightMath();
  public LimelightDataDemo() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("pipeline", subsystem.getPipeline());
    SmartDashboard.putBoolean("validTarget", subsystem.validTarget());
    SmartDashboard.putNumber("latency", subsystem.getLatency());
    SmartDashboard.putNumber("distance", math.calculateDistance());
    SmartDashboard.putNumber("angleV", math.calculateAngleToTargetV());
    SmartDashboard.putNumber("angleH", math.calculateAngleToTargetH());
    subsystem.setPipeline(LimelightEnum.valueOf(SmartDashboard.getString("pipeline", "TowerTarget")));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
