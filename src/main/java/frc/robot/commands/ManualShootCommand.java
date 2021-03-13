package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;


public class ManualShootCommand extends CommandBase {
    // Called when the command is initially scheduled.
    //Has a random speed
  @Override
  public void initialize() {
    System.out.println("Manual Shoot Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    XboxMap.shoot();
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
