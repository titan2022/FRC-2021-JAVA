package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ControlPanelMap;
import frc.robot.config.OI;
import frc.robot.config.XboxMap;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShooterCommand {
    private double g = 9.8;
    private double SHOOTER_ANGLE_RADIANS = Math.PI/4;
    private double HEIGHT_METERS = 3;
    private double INITIAL_DISTANCE_TO_TARGET_METERS = 9.144;

    private ShooterSubsystem shooterSubsystem;
    private SwerveDriveOdometryCommand swerveDriveOdometryCommand;

    public ManualShooterCommand(ShooterSubsystem shooterSubsystem, SwerveDriveOdometryCommand swerveDriveOdometryCommand){
      addRequirements(shooterSubsystem);
      this.shooterSubsystem = shooterSubsystem;
      this.swerveDriveOdometryCommand = swerveDriveOdometryCommand;
    }
    @Override
    public void initialize() {
      System.out.println("Manual Shoot Command Started");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(XboxMap.shoot())
      {
        double velocityOfBall = OI.xinmotek1.getY();
        shooterSubsystem.setOutput(velocityOfBall);
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
