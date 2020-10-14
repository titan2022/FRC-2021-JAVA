/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.Constants;
public class AutoAimCommand extends CommandBase {
  /**
   * Creates a new AutoAimCommand.
   */
  private AutoAimSubsystem subsystem;
  public XboxController xbox; //placeholder for dedicated xbox class
  public AutoAimCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem = new AutoAimSubsystem();
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading_error = -AutoAimSubsystem.x;
    double steeringAdjust = 0;
    if(AutoAimSubsystem.x> 1.0)
    {
       steeringAdjust = Constants.KpAim* heading_error - Constants.min_command;
    }
    else if(AutoAimSubsystem.x< 1.0)
    {
        steeringAdjust =  Constants.KpAim* heading_error + Constants.min_command;
    }
    AutoAimSubsystem.calculateDistance();
    AutoAimSubsystem.calculateAngleToTarget();
    double distanceError = Constants.desiredDistance - AutoAimSubsystem.distance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(xbox.getAButtonPressed()==true)
    {
      return false;
    }
    else{
      return true;
    }
  }
}
