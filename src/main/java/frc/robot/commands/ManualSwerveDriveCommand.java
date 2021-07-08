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
 * Manual field oriented velocity and orientation swerve drive manual control command.
 * Left joystick for translation and right joystick for orientation heading.
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

      // Convert into Y is forward and X is sideways
      double xFieldVelocity = XboxMap.translationX(); // For chassis speeds forward back is x and left right is y
      double yFieldVelocity = XboxMap.translationY();

      double headingRadians = 0;//navSub.getHeadingRadians(); // 0 to 2PI clockwise    
      
      //Trigonometric interpretation, not yet completed though
      // double xHeading = Math.cos(headingRadians);
      // double yHeading = Math.sin(headingRadians);

      // double deltaTheta = 0 - headingRadians; // Difference from heading zero

      // double xVelocity = xFieldVelocity * Math.cos(deltaTheta) - yFieldVelocity * Math.sin(deltaTheta);
      // double yVelocity = xFieldVelocity * Math.sin(deltaTheta) + yFieldVelocity * Math.cos(deltaTheta);

      // double velRelativeDotHeading = xHeading * xVelocity + yHeading * yVelocity;
      // double forwardVelocity = Math.sqrt(Math.pow(velRelativeDotHeading * xHeading, 2) + Math.pow(velRelativeDotHeading * yHeading, 2));


      // Coordinate transformation interpretation
      double xVelocity = Math.cos(headingRadians) * yFieldVelocity + Math.sin(headingRadians) * xFieldVelocity;
      double yVelocity = Math.cos(headingRadians) * xFieldVelocity + Math.sin(headingRadians) * yFieldVelocity;
      
      // Direction vector
      double xDirection = XboxMap.orientationX();
      double yDirection = XboxMap.orientationY();

      if (xDirection != 0 || yDirection != 0)
      {
        // atan2 returns -pi to pi where positive is counter clockwise
        targetAngleRadians = Math.atan2(yDirection, xDirection);
        // Convert from -pi to pi to 0 to 2pi with 0 being positive y axis
        targetAngleRadians = (-targetAngleRadians + 2 * Math.PI + Math.PI / 2) % (2 * Math.PI);
        //targetAngleRadians = Math.max(0, Math.min(Math.toRadians(28), targetAngleRadians)); // Limit if needed
      }

      swerveDriveSub.setVelocities(new ChassisSpeeds(xVelocity, yVelocity, 0)); //pid.calculate(headingRadians, targetAngleRadians)));

      SmartDashboard.putNumber("xFieldVel", xFieldVelocity);
      SmartDashboard.putNumber("yFieldVel", yFieldVelocity);
      SmartDashboard.putNumber("ipt x vel", xFieldVelocity);
      SmartDashboard.putNumber("ipt y vel", yFieldVelocity);
      SmartDashboard.putNumber("LF Rot Enc", swerveDriveSub.getRotatorEncoderCount(true, false));
      SmartDashboard.putNumber("LB Rot Enc", swerveDriveSub.getRotatorEncoderCount(true, true));
      SmartDashboard.putNumber("RF Rot Enc", swerveDriveSub.getRotatorEncoderCount(false, false));
      SmartDashboard.putNumber("RB Rot Enc", swerveDriveSub.getRotatorEncoderCount(false, true));
      SmartDashboard.putNumber("AHRS", Math.toDegrees(headingRadians));
      SmartDashboard.putNumber("ipt angle", Math.toDegrees(targetAngleRadians));
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
