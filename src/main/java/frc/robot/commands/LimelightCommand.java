/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
public class LimelightCommand extends CommandBase {
  /**
   * Creates a new LimelightCommand.
   */
  private LimelightSubsystem subsystem;
  public LimelightCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem = new LimelightSubsystem();
    
  }
  public double calculateDistance()
  {
    double distance = (Constants.targetHeight - Constants.limelightHeight) / Math.tan(Constants.limelightAngle + calculateAngleToTarget());
    return distance;
  }

  public double calculateAngleToTarget()
  {
    double nx = (1/160) *(LimelightSubsystem.getX() -159.5);  //normalized pixel values
    //double ny =(1/120) * (119.5 -y);
    double vpw = 2.0 *Math.tan(54/2);  //calculates horizontal fov
    //double vph = 2.0 *Math.tan(41/2);
    double xCoor = vpw/2 *nx;
    double angleToTarget = Math.atan(xCoor/1);
    return angleToTarget;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimelightSubsystem.validTarget())
    {
      calculateDistance();
    }
    else
    {
      DriverStation.reportError("Valid target not found", false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
