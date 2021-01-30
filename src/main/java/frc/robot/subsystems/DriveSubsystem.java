package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.sim.PhysicsSim;

/**
 * Differential Drive Subsystem
 */
public class DriveSubsystem extends SubsystemBase
{
  // TODO: add constants to file later
  public static final double ROBOT_DIAMETER = 42; // meter //TODO: Ask DI team for distance between wheels
  public static final double WHEEL_RADIUS = 3; // meters// TODO: Ask DI team for correct wheel radius
  public static final double METERS_PER_TICK = 0.1;

  // port numbers to be added later
  public static final int LEFT_PRIMARY_PORT = 1;
  public static final int LEFT_SECONDARY_PORT = 2;
  public static final int RIGHT_PRIMARY_PORT = 3;
  public static final int RIGHT_SECONDARY_PORT = 4;

  private static final int ENCODER_PORT = 1;

  // Physical and Simulated Hardware
  // These talon objects are also simulated
  public final WPI_TalonSRX leftPrimary = new WPI_TalonSRX(LEFT_PRIMARY_PORT)
    , leftSecondary =new WPI_TalonSRX(LEFT_SECONDARY_PORT)
    , rightPrimary = new WPI_TalonSRX(RIGHT_PRIMARY_PORT)
    , rightSecondary = new WPI_TalonSRX(RIGHT_SECONDARY_PORT);
  
  private static final double MAX_SPEED = 10; // meters/sec
  private static final int PEAK_CURRENT_LIMIT = 60;
  private static final int CONTINUOUS_CURRENT_LIMIT = 50;
  private static final double FULL_ACCEL_TIME = 0.75; // sec
  private static final double MAX_MOTOR_VEL = 4000; // ticks/ (100ms)
  private static final boolean PRIMARY_MOTOR_SENSOR_PHASE = true;

  

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem()
  {
    // TODO: Remove magic numbers
    // motor configuration block
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

    // Sets the direction that the talon will turn on the green LED when going 'forward'.
    leftPrimary.setSensorPhase(true);
    rightPrimary.setSensorPhase(true);

    // Current limits in amps
    leftPrimary.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);
    leftPrimary.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);
    leftPrimary.enableCurrentLimit(true);

    rightPrimary.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);
    rightPrimary.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);
    rightPrimary.enableCurrentLimit(true);

    /* TODO: Deal with motor controller faults once a physical robot is available for testing
    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();
    */
  }

  public DriveSubsystem(TalonSRXConfiguration leftConfig, TalonSRXConfiguration rightConfig)
  {
    this();

    leftPrimary.configAllSettings(leftConfig);
    leftSecondary.configAllSettings(leftConfig);
    rightPrimary.configAllSettings(rightConfig);
    rightSecondary.configAllSettings(rightConfig);
  }

  public DriveSubsystem(TalonSRXConfiguration leftConfig, TalonSRXConfiguration rightConfig, boolean simulated)
  {
    this(leftConfig, rightConfig);
    if(simulated) enableSimulation();
  }

  public DriveSubsystem(boolean simulated)
  {
    this();
    if(simulated) enableSimulation();
  }

  private void enableSimulation()
  {
    PhysicsSim.getInstance().addTalonSRX(leftPrimary, FULL_ACCEL_TIME, MAX_MOTOR_VEL, PRIMARY_MOTOR_SENSOR_PHASE);
    PhysicsSim.getInstance().addTalonSRX(rightPrimary, FULL_ACCEL_TIME, MAX_MOTOR_VEL, PRIMARY_MOTOR_SENSOR_PHASE);
    PhysicsSim.getInstance().addTalonSRX(leftSecondary, FULL_ACCEL_TIME, MAX_MOTOR_VEL);
    PhysicsSim.getInstance().addTalonSRX(rightSecondary, FULL_ACCEL_TIME, MAX_MOTOR_VEL);
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
    if (mode == ControlMode.Velocity && leftOutputValue > MAX_SPEED) {
      leftOutputValue = MAX_SPEED;
    }

    if (mode == ControlMode.Velocity && rightOutputValue > MAX_SPEED) {
      rightOutputValue = MAX_SPEED;
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
   * Stops the motors.
   */
  public void stop() {
    leftPrimary.set(ControlMode.PercentOutput, 0);
    rightPrimary.set(ControlMode.PercentOutput, 0);
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

  /**
   * Gets the distance from a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Distance from specified primary motor.
   */
  public double getEncoderDist(boolean useLeft) {
    return getEncoderCount(useLeft) * METERS_PER_TICK;
  }
  
  /**
   * Gets current velocity of a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Current velocity of a specified primary motor.
   */
  public double getEncoderVelocity(boolean useLeft) {
    if (useLeft)
    {
      return leftPrimary.getSelectedSensorVelocity(0) * METERS_PER_TICK;
    }
    else
    {
      return rightPrimary.getSelectedSensorVelocity(0) * METERS_PER_TICK;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
    
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    driveSim.setInputs(
        m_leftLeader.get() * RobotController.getInputVoltage(),
        -m_rightLeader.get() * RobotController.getInputVoltage());
        driveSim.update(0.02);

    drive.leftPrimary.setDistance(driveSim.getLeftPositionMeters());
    drive.leftPrimary.setRate(driveSim.getLeftVelocityMetersPerSecond());
    drive.rightPrimary.setDistance(driveSim.getRightPositionMeters());
    drive.rightPrimary.setRate(driveSim.getRightVelocityMetersPerSecond());
  }
}