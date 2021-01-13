package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// using WPILib's docs' example from:
// https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html

/**
 * Differential Drive Subsystem
 */
public class DriveSubsystem extends SubsystemBase {

  // port numbers to be added later
  // TODO: add constants to file later

  private final static int LEFT_PRIMARY_PORT = 1;
  private final static int LEFT_SECONDARY_PORT = 2;
  private final static int RIGHT_PRIMARY_PORT = 3;
  private final static int RIGHT_SECONDARY_PORT = 4;

  private TalonSRX leftPrimary, leftSecondary, rightPrimary, rightSecondary;

  private final static double MAX_SPEED = 10; // meters/sec

  //TODO: add encoders?

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // motor initialization block

    leftPrimary = new TalonSRX(LEFT_PRIMARY_PORT);
    leftSecondary = new TalonSRX(LEFT_SECONDARY_PORT);
    rightPrimary = new TalonSRX(RIGHT_PRIMARY_PORT);
    rightSecondary = new TalonSRX(RIGHT_SECONDARY_PORT);
    
    leftPrimary.configFactoryDefault();
    leftSecondary.configFactoryDefault();
    rightPrimary.configFactoryDefault();
    rightSecondary.configFactoryDefault();

    rightPrimary.setInverted(false);
    rightSecondary.setInverted(false);

    leftPrimary.setInverted(false);
    leftSecondary.setInverted(false);

    leftSecondary.follow(leftPrimary);
    rightSecondary.follow(rightPrimary);
    
    //Sets the direction that the talon will turn on the green LED when going 'forward'.
    leftPrimary.setSensorPhase(true);
    rightPrimary.setSensorPhase(true);

    //Current limits in amps
    leftPrimary.configPeakCurrentLimit(60);
		leftPrimary.configContinuousCurrentLimit(50);
		leftPrimary.enableCurrentLimit(true);
		
		rightPrimary.configPeakCurrentLimit(60);
		rightPrimary.configContinuousCurrentLimit(50);
		rightPrimary.enableCurrentLimit(true);
  }

  public DriveSubsystem(TalonSRXConfiguration leftConfig, TalonSRXConfiguration rightConfig) {
    this();

    leftPrimary.configAllSettings(leftConfig);
    leftSecondary.configAllSettings(leftConfig);
    rightPrimary.configAllSettings(rightConfig);
    rightSecondary.configAllSettings(rightConfig);
  }

  /**
   * Returns the current maximum drive speed in meters per second.
   * @return Maximum drive speed in meters per second.
   */
  public double getMaxSpeed() {
    return MAX_SPEED;
  }

  /**
   * Sets motor outputs using specified control mode
   * @param mode a ControlMode enum
   * @param leftOutputValue left side output value for ControlMode
   * @param rightOutputValue right side output value for ControlMode
   */
  public void setOutput(ControlMode mode, double leftOutputValue, double rightOutputValue)
  {
    if(mode == ControlMode.Velocity && leftOutputValue > MAX_SPEED)
    {
      leftOutputValue = MAX_SPEED;
    }

    if(mode == ControlMode.Velocity && rightOutputValue > MAX_SPEED)
    {
      rightOutputValue = MAX_SPEED;
    }

    //TODO: is check the current usage from Power Subsystem to restrict overcurrent
    leftPrimary.set(mode, leftOutputValue);
    rightPrimary.set(mode, rightOutputValue);
  }

  /**
   * Enables brake.
   */
  public void enableBrakes() {
    leftPrimary.setNeutralMode(NeutralMode.Brake);
    rightPrimary.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Disables brake.
   */
  public void disableBrakes() {
    leftPrimary.setNeutralMode(NeutralMode.Coast);
    rightPrimary.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Stops the motors.
   */
  public void stop() {
    leftPrimary.set(ControlMode.PercentOutput, 0);
    rightPrimary.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}