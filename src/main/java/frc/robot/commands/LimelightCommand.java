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
public class LimelightCommand extends CommandBase {
  /**
   * Creates a new AutoAimCommand.
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    LimelightSubsystem.steering();
    LimelightSubsystem.distance();
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
