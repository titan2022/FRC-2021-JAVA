package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;

public class AssisstedShooterCommand extends CommandBase{
    // Called when the command is initially scheduled.
    //Has a random speed
    @Override
    public void initialize() {
      System.out.println("Assisted Shoot Command Started");
    }
    public double getExitAngle()
    {
        double exitAngle = 0;
        exitAngle = Math.atan((height + (1/2 * g* t * t))/distance);
        return exitAngle;
    }

    public double getInitialVelocity()
    {
        double initialVelocity = 0;
        initialVelocity = (distance/Math.cos(getExitAngle())) * t;
        return initialVelocity;
    }

    public double optimizeAngle()
    {
        double initialAngleMax = 0.0;
        initialAngleMax = Math.atan(height/distance);
        return initialAngleMax;
    }

    public double optimizeVelocity()
    {
      double initialVelocityMax = 0.0;
      initialVelocityMax = distance/Math.cos(optimizeAngle());
      return initialVelocityMax;
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
  