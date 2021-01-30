package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.subsystems.sim.PhysicsSim;

/**
 * Differential Drive Subsystem
 */
public class DriveSubsystem extends SubsystemBase
{
  // Physical parameters
  public static final double ROBOT_TRACK_WIDTH = 42; // meter // TODO: Ask DI team for distance between wheels
  public static final double WHEEL_RADIUS = 3; // meters // TODO: Ask DI team for correct wheel radius
  public static final double ENCODER_TICKS = 4096; // TODO: Figure out the number of ticks in our encoders
  public static final double METERS_PER_TICK = WHEEL_RADIUS * Math.PI / ENCODER_TICKS; // TODO: Recompute using wheel circumference / encoder ticks
  public static final double GEARING_REDUCTION = 7.29; // TODO: Get the correct gearing ratio

  // Port numbers to be added later
  private static final int LEFT_PRIMARY_PORT = 1;
  private static final int LEFT_SECONDARY_PORT = 2;
  private static final int RIGHT_PRIMARY_PORT = 3;
  private static final int RIGHT_SECONDARY_PORT = 4;

  private static final int ENCODER_PORT = 1;

  // Motor and sensor inversions
  private static final boolean LEFT_PRIMARY_INVERTED = false;
  private static final boolean LEFT_SECONDARY_INVERTED = false;
  private static final boolean RIGHT_PRIMARY_INVERTED = false;
  private static final boolean RIGHT_SECONDARY_INVERTED = false;
  private static final boolean LEFT_PRIMARY_MOTOR_SENSOR_PHASE = true;
  private static final boolean RIGHT_PRIMARY_MOTOR_SENSOR_PHASE = true;

  // Physical limits
  private static final double MAX_SPEED = 10; // meters/sec
  private static final int PEAK_CURRENT_LIMIT = 60;
  private static final int CONTINUOUS_CURRENT_LIMIT = 50;

  // Phoenix Physics Sim Variables
  private static final double FULL_ACCEL_TIME = 0.75; // sec
  private static final double MAX_MOTOR_VEL = 4000; // ticks/(100ms)

  // Physical and Simulated Hardware
  // These talon objects are also simulated
  private static final WPI_TalonSRX leftPrimary = new WPI_TalonSRX(LEFT_PRIMARY_PORT)
    , leftSecondary = new WPI_TalonSRX(LEFT_SECONDARY_PORT)
    , rightPrimary = new WPI_TalonSRX(RIGHT_PRIMARY_PORT)
    , rightSecondary = new WPI_TalonSRX(RIGHT_SECONDARY_PORT);

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

  // Physics timer
  private Timer simTime;

  // Create the simulation model of our drivetrain.
  private static DifferentialDrivetrainSim driveSim;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem()
  {
    // motor configuration block
    leftPrimary.configFactoryDefault();
    leftSecondary.configFactoryDefault();
    rightPrimary.configFactoryDefault();
    rightSecondary.configFactoryDefault();

    rightPrimary.setInverted(RIGHT_PRIMARY_INVERTED);
    rightSecondary.setInverted(RIGHT_SECONDARY_INVERTED);

    leftPrimary.setInverted(LEFT_PRIMARY_INVERTED);
    leftSecondary.setInverted(LEFT_SECONDARY_INVERTED);

    leftSecondary.follow(leftPrimary);
    rightSecondary.follow(rightPrimary);

    // Sets the direction that the talon will turn on the green LED when going 'forward'.
    leftPrimary.setSensorPhase(LEFT_PRIMARY_MOTOR_SENSOR_PHASE);
    rightPrimary.setSensorPhase(RIGHT_PRIMARY_MOTOR_SENSOR_PHASE);

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
    if (simulated) enableSimulation();
  }

  public DriveSubsystem(boolean simulated)
  {
    this();
    if (simulated) enableSimulation();
  }

  private void enableSimulation()
  {
    PhysicsSim.getInstance().addTalonSRX(leftPrimary, FULL_ACCEL_TIME, MAX_MOTOR_VEL, LEFT_PRIMARY_MOTOR_SENSOR_PHASE);
    PhysicsSim.getInstance().addTalonSRX(rightPrimary, FULL_ACCEL_TIME, MAX_MOTOR_VEL, RIGHT_PRIMARY_MOTOR_SENSOR_PHASE);
    PhysicsSim.getInstance().addTalonSRX(leftSecondary, FULL_ACCEL_TIME, MAX_MOTOR_VEL);
    PhysicsSim.getInstance().addTalonSRX(rightSecondary, FULL_ACCEL_TIME, MAX_MOTOR_VEL);

    driveSim = new DifferentialDrivetrainSim(
      // Create a linear system from our characterization gains.
      LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
      DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.  // TODO: Set the correct type of motor
      GEARING_REDUCTION,
      ROBOT_TRACK_WIDTH,
      WHEEL_RADIUS,
      VecBuilder.fill(X_MEAS_NOISE, Y_MEAS_NOISE, HEADING_MEAS_NOISE, LEFT_VEL_MEAS_NOISE, RIGHT_VEL_MEAS_NOISE, LEFT_POS_MEAS_NOISE, RIGHT_POS_MEAS_NOISE));

    simTime = new Timer();
    simTime.start();
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
    // driveSim.setInputs(
    //     m_leftLeader.get() * RobotController.getInputVoltage(),
    //     -m_rightLeader.get() * RobotController.getInputVoltage());
    driveSim.setInputs(leftPrimary.getMotorOutputVoltage(), rightPrimary.getMotorOutputVoltage());
    driveSim.update(simTime.get());
    simTime.reset();

    leftPrimary.setSelectedSensorPosition(driveSim.getLeftPositionMeters());
    rightPrimary.setSelectedSensorPosition(driveSim.getRightPositionMeters());
  }
}