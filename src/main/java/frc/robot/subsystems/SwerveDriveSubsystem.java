package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.subsystems.sim.PhysicsSim;

public class SwerveDriveSubsystem extends SubsystemBase
{
  // Physical parameters
  public static final double ROBOT_TRACK_WIDTH = 26.75/39.37; // meter
  public static final double WHEEL_RADIUS = 6/39.37; // meters
  public static final double ENCODER_TICKS = 4096; // Ticks/rotation of CTREMagEncoder
  public static final double METERS_PER_TICK = WHEEL_RADIUS * 2 * Math.PI / ENCODER_TICKS;
  public static final double GEARING_REDUCTION = 7.29; // TODO: Get the correct gearing ratio
    
  // Port numbers to be added later
  private static final int LEFT_PRIMARY_PORT = 1;
  private static final int LEFT_SECONDARY_PORT = 2;
  private static final int RIGHT_PRIMARY_PORT = 3;
  private static final int RIGHT_SECONDARY_PORT = 4;
  private static final int LEFT_PRIMARY_ROTATOR_PORT = 5;
  private static final int LEFT_SECONDARY_ROTATOR_PORT = 6;
  private static final int RIGHT_PRIMARY_ROTATOR_PORT = 7;
  private static final int RIGHT_SECONDARY_ROTATOR_PORT = 8;

  private static final int ENCODER_PORT = 1;

  // Motor and sensor inversions
  private static final boolean LEFT_PRIMARY_INVERTED = false;
  private static final boolean LEFT_SECONDARY_INVERTED = false;
  private static final boolean LEFT_PRIMARY_ROTATOR_INVERTED = false;
  private static final boolean LEFT_SECONDARY_ROTATOR_ARY_INVERTED = false;
  private static final boolean RIGHT_PRIMARY_INVERTED = false;
  private static final boolean RIGHT_SECONDARY_INVERTED = false;
  private static final boolean RIGHT_PRIMARY_ROTATOR_INVERTED = false;
  private static final boolean RIGHT_SECONDARY_ROTATOR_INVERTED = false;

  private static final boolean LEFT_PRIMARY_MOTOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_PRIMARY_MOTOR_SENSOR_PHASE = false;

  // Physical limits of motors that create translational motion
  private static final double MAX_SPEED = 10; // meters/sec
  private static final int PEAK_CURRENT_LIMIT = 60;
  private static final int CONTINUOUS_CURRENT_LIMIT = 50;

  // Physical limits of motors that rotate the wheel
  //private static final double MAX_SPEED = 10; // meters/sec
  //private static final int PEAK_CURRENT_LIMIT = 60;
  //private static final int CONTINUOUS_CURRENT_LIMIT = 50;

  // Phoenix Physics Sim Variables for motors that create translational motion
  private static final double FULL_ACCEL_TIME = 0.75; // sec
  private static final double MAX_MOTOR_VEL = 4000; // ticks/(100ms)

  // Phoenix Physics Sim Variables for motors that rotate wheels
  //private static final double FULL_ACCEL_TIME = 0.75; // sec
  //private static final double MAX_MOTOR_VEL = 4000; // ticks/(100ms)
  
  // Physical and Simulated Hardware
  // These talon objects are also simulated
  private static final WPI_TalonSRX leftPrimary = new WPI_TalonSRX(LEFT_PRIMARY_PORT)
    , leftSecondary = new WPI_TalonSRX(LEFT_SECONDARY_PORT)
    , rightPrimary = new WPI_TalonSRX(RIGHT_PRIMARY_PORT)
    , rightSecondary = new WPI_TalonSRX(RIGHT_SECONDARY_PORT)
    , leftPrimaryRotator = new WPI_TalonSRX(LEFT_PRIMARY_ROTATOR_PORT)
    , leftSecondaryRotator = new WPI_TalonSRX(LEFT_SECONDARY_ROTATOR_PORT)
    , rightPrimaryRotator = new WPI_TalonSRX(RIGHT_PRIMARY_ROTATOR_PORT)
    , rightSecondaryRotator = new WPI_TalonSRX(RIGHT_SECONDARY_ROTATOR_PORT);

  // Physics simulation
  // Feedforward gain constants (from the characterization tool)
  private static final double KvLinear = 1.98;
  private static final double KaLinear = 0.2;
  private static final double KvAngular = 1.5;
  private static final double KaAngular = 0.3;

  // Standard deviation for measurement noise
  private static final double X_MEAS_NOISE = 0.001; // meter
  private static final double Y_MEAS_NOISE = 0.001; // meter
  private static final double HEADING_MEAS_NOISE = 0.001; // radian
  private static final double LEFT_VEL_MEAS_NOISE = 0.1; // meter / second
  private static final double RIGHT_VEL_MEAS_NOISE = 0.1; // meter / second
  private static final double LEFT_POS_MEAS_NOISE = 0.005; // meter
  private static final double RIGHT_POS_MEAS_NOISE = 0.005; // meter

  /**
   * Creates a new SwerveSubsystem.
   */
  public SwerveDriveSubsystem() 
  {
    // motor configuration block
    leftPrimary.configFactoryDefault();
    leftSecondary.configFactoryDefault();
    rightPrimary.configFactoryDefault();
    rightSecondary.configFactoryDefault();
    leftPrimaryRotator.configFactoryDefault();
    leftSecondaryRotator.configFactoryDefault();
    rightPrimaryRotator.configFactoryDefault();
    rightSecondaryRotator.configFactoryDefault();

    rightPrimary.setInverted(RIGHT_PRIMARY_INVERTED);
    rightSecondary.setInverted(RIGHT_SECONDARY_INVERTED);
    rightPrimaryRotator.setInverted(RIGHT_PRIMARY_ROTATOR_INVERTED);
    rightSecondaryRotator.setInverted(RIGHT_SECONDARY_ROTATOR_INVERTED);

    leftPrimary.setInverted(LEFT_PRIMARY_INVERTED);
    leftSecondary.setInverted(LEFT_SECONDARY_INVERTED);
    leftPrimaryRotator.setInverted(LEFT_PRIMARY_ROTATOR_INVERTED);
    leftSecondaryRotator.setInverted(LEFT_SECONDARY_ROTATOR_ARY_INVERTED);

    //Add if rotators follow
    leftSecondary.follow(leftPrimary);
    rightSecondary.follow(rightPrimary);
    leftSecondaryRotator.follow(leftPrimaryRotator);
    rightSecondaryRotator.follow(rightPrimaryRotator);

    // Sets the direction that the talon will turn on the green LED when going 'forward'.
    leftPrimary.setSensorPhase(LEFT_PRIMARY_MOTOR_SENSOR_PHASE);
    rightPrimary.setSensorPhase(RIGHT_PRIMARY_MOTOR_SENSOR_PHASE);
    //Might need to add more for rotator motors. 

    // Current limits in amps
    leftPrimary.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);
    leftPrimary.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);
    leftPrimary.enableCurrentLimit(true);

    rightPrimary.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);
    rightPrimary.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);
    rightPrimary.enableCurrentLimit(true);
    // Might need more for rotator motors

    /* TODO: Deal with motor controller faults once a physical robot is available for testing
    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();
    */
  }

  //Might need extra parameters for rotator motors
  public SwerveDriveSubsystem(TalonSRXConfiguration leftConfig, TalonSRXConfiguration rightConfig)
  {
    this();

    leftPrimary.configAllSettings(leftConfig);
    leftSecondary.configAllSettings(leftConfig);
    rightPrimary.configAllSettings(rightConfig);
    rightSecondary.configAllSettings(rightConfig);
  }

  /**
   * Returns the current maximum drive speed in meters per second.
   * 
   * @return Maximum drive speed in meters per second.
   */
  public double getMaxSpeed()
  {
    return MAX_SPEED;
  }

  /**
   * Sets motor outputs using specified control mode
   * 
   * @param mode             a ControlMode enum
   * @param leftOutputValue  left side output value for ControlMode
   * @param rightOutputValue right side output value for ControlMode
   */
  public void setOutput(ControlMode mode, double leftOutputValue, double rightOutputValue) {

    if (mode == ControlMode.Velocity) {

      if (leftOutputValue > MAX_SPEED) {
        leftOutputValue = MAX_SPEED;
      }

      if (rightOutputValue > MAX_SPEED) {
        rightOutputValue = MAX_SPEED;
      }

    }

    // TODO: is check the current usage from Power Subsystem to restrict overcurrent
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
   * Enables brake for rotator motors.
   */
  public void enableRotatorBrakes() {
    leftPrimaryRotator.setNeutralMode(NeutralMode.Brake);
    rightPrimaryRotator.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Disables brake for rotator motors.
   */
  public void disableRotatorBrakes() {
    leftPrimaryRotator.setNeutralMode(NeutralMode.Coast);
    rightPrimaryRotator.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Stops the motors.
   */
  public void stop() {
    leftPrimary.set(ControlMode.PercentOutput, 0);
    rightPrimary.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Stops the rotator motors.
   */
  public void stopRotators() {
    leftPrimaryRotator.set(ControlMode.PercentOutput, 0);
    rightPrimaryRotator.set(ControlMode.PercentOutput, 0);
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getEncoderCount(boolean useLeft)
  {
    if (useLeft)
    {
      return leftPrimary.getSelectedSensorPosition(ENCODER_PORT);
    }
    else
    {
      return rightPrimary.getSelectedSensorPosition(ENCODER_PORT);
    }
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getRotatorEncoderCount(boolean useLeft)
  {
    if (useLeft)
    {
      return leftPrimaryRotator.getSelectedSensorPosition(ENCODER_PORT);
    }
    else
    {
      return rightPrimaryRotator.getSelectedSensorPosition(ENCODER_PORT);
    }
  }

  /**
   * Gets the distance from a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Distance from specified primary motor.
   */
  public double getEncoderDist(boolean useLeft) {
    return getEncoderCount(useLeft) * METERS_PER_TICK;
  }

  /**
   * Gets the amount of rotation from a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Rotation of a specified primary motor.
   */
  public double getRotatorEncoderDist(boolean useLeft) {
    return getRotatorEncoderCount(useLeft) * METERS_PER_TICK / WHEEL_RADIUS;
  }

  /**
   * Gets current velocity of a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Current velocity of a specified primary motor.
   */
  public double getEncoderVelocity(boolean useLeft) {
    if (useLeft)
    {
      return leftPrimary.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK;
    }
    else
    {
      return rightPrimary.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK;
    }
  }

  /**
   * Gets current rotational velocity of a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Current rotational velocity of a specified primary motor.
   */
  public double getRotatorEncoderVelocity(boolean useLeft) {
    if (useLeft)
    {
      return leftPrimary.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK / WHEEL_RADIUS;
    }
    else
    {
      return rightPrimary.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK / WHEEL_RADIUS;
    }
  }

  /**
   * Gets FPGA time from robot and converts it to seconds.
   * 
   * @return FPGA time in seconds.
   */
  public double getRobotTime() {

    return RobotController.getFPGATime() / 1e6;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
