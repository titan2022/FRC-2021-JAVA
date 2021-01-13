/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision.demos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightEnum;

public class LimelightDataDemo extends CommandBase {
  /**
   * Creates a new LimelightDataDemo.
   */
  Limelight limelight = new Limelight();
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
    SmartDashboard.putString("pipeline", limelight.pipeline());
    SmartDashboard.putBoolean("validTarget", limelight.validTarget());
    SmartDashboard.putNumber("latency", limelight.latency());
    SmartDashboard.putNumber("distance", limelight.calculateDistance());
    SmartDashboard.putNumber("angleV", limelight.calculateAngleToTargetV());
    SmartDashboard.putNumber("angleH", limelight.calculateAngleToTargetH());
    limelight.setPipeline(LimelightEnum.valueOf(SmartDashboard.getString("pipeline", "TowerTarget")));
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
