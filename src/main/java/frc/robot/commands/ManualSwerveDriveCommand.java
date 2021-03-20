package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;
import frc.robot.motion.control.PIDConfig;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

/**
 * 
 */
public class ManualSwerveDriveCommand extends CommandBase {
  private SwerveDriveSubsystem swerveDriveSub;
  private NavigationSubsystem navSub;
  private PIDController pid;
  private boolean brakeState = false;
  private double targetAngleRadians = 0;

  /** Creates a new ManualDifferentialDriveCommand. 
   *  Joystick inputs are calculated in 0 to 360
  */
  public ManualSwerveDriveCommand(SwerveDriveSubsystem swerveDriveSub, NavigationSubsystem navSub, PIDConfig pidConfig) { // TODO: Maybe consider changing from PIDConfigs to {@link PIDController}
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSub);
    this.swerveDriveSub = swerveDriveSub;
    this.navSub = navSub;

    pid = new PIDController(pidConfig.kP, pidConfig.kI, pidConfig.kD);
    pid.enableContinuousInput(0, 2 * Math.PI + 1e-22); // PID controller should not account for gear reductions
    pid.setIntegratorRange(pidConfig.INTEGRATION_MIN, pidConfig.INTEGRATION_MAX);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Manual Swerve Drive Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    brakeState = XboxMap.toggleBrakes() ? !brakeState : brakeState;

    if (brakeState) {
      swerveDriveSub.enableBrakes();
    }
    else { 
      swerveDriveSub.disableBrakes();

      double headingRadians = navSub.getHeadingRadians(); // 0 to 2PI clockwise
      double xHeading = Math.cos(headingRadians);
      double yHeading = Math.sin(headingRadians);

      // Convert into Y is forward and X is sideways
      double xFieldVelocity = XboxMap.leftX(); // For chassis speeds forward back is x and left right is y
      double yFieldVelocity = XboxMap.leftY();
      double deltaTheta = 0 - headingRadians; // Difference from heading zero
      
      /* Trigonometric interpretation, not yet completed though
      double xVelocity = xFieldVelocity * Math.cos(deltaTheta) - yFieldVelocity * Math.sin(deltaTheta);
      double yVelocity = xFieldVelocity * Math.sin(deltaTheta) + yFieldVelocity * Math.cos(deltaTheta);

      double velRelativeDotHeading = xHeading * xVelocity + yHeading * yVelocity;
      double forwardVelocity = Math.sqrt(Math.pow(velRelativeDotHeading * xHeading, 2) + Math.pow(velRelativeDotHeading * yHeading, 2));
      */

      double xVelocity = Math.cos(headingRadians) * yFieldVelocity + Math.sin(headingRadians) * xFieldVelocity;
      double yVelocity = Math.cos(headingRadians) * xFieldVelocity + Math.sin(headingRadians) * yFieldVelocity;

      
      // Direction vector
      double xDirection = XboxMap.rightX();
      double yDirection = XboxMap.rightY();

      if (xDirection != 0 && yDirection != 0)
      {
        // atan2 returns -pi to pi where positive is counter clockwise
        targetAngleRadians = Math.atan2(XboxMap.rightY(), XboxMap.rightX());
        // Convert from -pi to pi to 0 to 2pi with 0 being positive y axis
        targetAngleRadians = (-targetAngleRadians + 2 * Math.PI + Math.PI / 2) % (2 * Math.PI);
        //targetAngleRadians = Math.max(0, Math.min(Math.toRadians(28), targetAngleRadians)); // Limit if needed
      }

      swerveDriveSub.setOutput(new ChassisSpeeds(xVelocity, yVelocity, pid.calculate(headingRadians, targetAngleRadians)));

      SmartDashboard.putNumber("xFieldVelocity", xFieldVelocity);
      SmartDashboard.putNumber("yFieldVelocity", yFieldVelocity);
      SmartDashboard.putNumber("Left Front Rotator Encoder Value", swerveDriveSub.getRotatorEncoderCount(true, false));
      SmartDashboard.putNumber("Left Back Rotator Encoder Value", swerveDriveSub.getRotatorEncoderCount(true, true));
      SmartDashboard.putNumber("Right Front Rotator Encoder Value", swerveDriveSub.getRotatorEncoderCount(false, false));
      SmartDashboard.putNumber("Right Back Rotator Encoder Value", swerveDriveSub.getRotatorEncoderCount(false, true));
      SmartDashboard.putNumber("AHRS", Math.toDegrees(headingRadians));
      SmartDashboard.putNumber("target angle", Math.toDegrees(targetAngleRadians));
      SmartDashboard.putNumber("Yaw", navSub.getYaw());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveSub.stop();
		System.out.println("Manual Swerve Drive Command Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
