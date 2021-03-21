package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;
import frc.robot.subsystems.ShooterSubsystem;

public class InterstellarAccuracyCommand extends CommandBase{
    // Called when the command is initially scheduled.
    //Has a random speed
    private double g = 9.8;
    private double SHOOTER_ANGLE_RADIANS = Math.PI/4;
    private double HEIGHT_METERS = 3;
    private double INITIAL_DISTANCE_TO_TARGET_METERS = 9.144;

    private ShooterSubsystem shooterSubsystem;
    private SwerveDriveOdometryCommand swerveDriveOdometryCommand;

    public InterstellarAccuracyCommand(ShooterSubsystem shooterSubsystem, SwerveDriveOdometryCommand swerveDriveOdometryCommand){
      addRequirements(shooterSubsystem);
      this.shooterSubsystem = shooterSubsystem;
      this.swerveDriveOdometryCommand = swerveDriveOdometryCommand;
    }
    @Override
    public void initialize() {
      System.out.println("Assisted Shoot Command Started");
    }

    // public double optimizeVelocity()
    // {
    //   double initialVelocityMax = 0.0;
    //   initialVelocityMax = getDistance()/Math.cos(optimizeAngle());
    //   return initialVelocityMax;
    // }

    public double getVelocity(double x, double y, double shooterAngle){
      return Math.sqrt((g*x*x)/(2*Math.cos(shooterAngle)*(x*Math.tan(shooterAngle)-y)));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(XboxMap.shoot())
      {
        shooterSubsystem.setOutput(getVelocity(INITIAL_DISTANCE_TO_TARGET_METERS-swerveDriveOdometryCommand.getY(), HEIGHT_METERS, SHOOTER_ANGLE_RADIANS));
      }
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
  