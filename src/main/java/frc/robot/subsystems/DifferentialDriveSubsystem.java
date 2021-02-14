package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.subsystems.sim.PhysicsSim;

/**
 * Differential Drive Subsystem
 */
public class DifferentialDriveSubsystem extends SubsystemBase implements DriveSubsystem
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

  private static final int ENCODER_PORT = 0;

  // Motor and sensor inversions
  private static final boolean LEFT_PRIMARY_INVERTED = false;
  private static final boolean LEFT_SECONDARY_INVERTED = false;
  private static final boolean RIGHT_PRIMARY_INVERTED = false;
  private static final boolean RIGHT_SECONDARY_INVERTED = false;
  private static final boolean LEFT_PRIMARY_MOTOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_PRIMARY_MOTOR_SENSOR_PHASE = false;

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

  // Kinematics
  private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(ROBOT_TRACK_WIDTH);

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
  private double simPrevT = 0;

  // Create the simulation model of our drivetrain.
  private DifferentialDrivetrainSim driveSim;

  public DifferentialDriveSubsystem(TalonSRXConfiguration leftConfig, TalonSRXConfiguration rightConfig, boolean simulated)
  {
    setFactoryMotorConfig();

    if(leftConfig != null && rightConfig != null)
    {
      leftPrimary.configAllSettings(leftConfig);
      leftSecondary.configAllSettings(leftConfig);
      rightPrimary.configAllSettings(rightConfig);
      rightSecondary.configAllSettings(rightConfig);
    }

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
    if (simulated) enableSimulation();
  }

  /**
   * Creates a new DifferentialDriveSubsystem.
   */
  public DifferentialDriveSubsystem()
  {
    this(null, null, false);
  }

  public DifferentialDriveSubsystem(TalonSRXConfiguration leftConfig, TalonSRXConfiguration rightConfig)
  {
    this(leftConfig, rightConfig, false);
  }

  public DifferentialDriveSubsystem(boolean simulated)
  {
    this(null, null, simulated);
  }

  private void setFactoryMotorConfig()
  {
    leftPrimary.configFactoryDefault();
    leftSecondary.configFactoryDefault();
    rightPrimary.configFactoryDefault();
    rightSecondary.configFactoryDefault();
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
      DCMotor.getFalcon500(2),       // 2 Falcon500 motors on each side of the drivetrain.  // TODO: Set the correct type of motor
      GEARING_REDUCTION,
      ROBOT_TRACK_WIDTH,
      WHEEL_RADIUS,
      VecBuilder.fill(X_MEAS_NOISE, Y_MEAS_NOISE, HEADING_MEAS_NOISE, LEFT_VEL_MEAS_NOISE, RIGHT_VEL_MEAS_NOISE, LEFT_POS_MEAS_NOISE, RIGHT_POS_MEAS_NOISE));
  
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
   * @implNote Velocity, Motion Magic, Motion Profile, and Position control modes use native sensor units
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

  @Override
  public void setVelocities(ChassisSpeeds velocities) {
    DifferentialDriveWheelSpeeds leftRightSpeeds = kinematics.toWheelSpeeds(velocities);
    setOutput(ControlMode.Velocity, leftRightSpeeds.leftMetersPerSecond / METERS_PER_TICK, leftRightSpeeds.rightMetersPerSecond / METERS_PER_TICK);
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
      return leftPrimary.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK;
    }
    else
    {
      return rightPrimary.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK;
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

  // Simulation Interface Methods
  public DifferentialDrivetrainSim getDriveSim() { // TODO: throw exception when the DifferentialDriveSubsystem is not being simulated
    return driveSim;
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

    // TODO: Delete this code when proven correct
    // driveSim.setInputs(
    //     m_leftLeader.get() * RobotController.getInputVoltage(),
    //     -m_rightLeader.get() * RobotController.getInputVoltage());
    driveSim.setInputs(leftPrimary.getMotorOutputVoltage(), rightPrimary.getMotorOutputVoltage());
    driveSim.update(getRobotTime() - simPrevT);
    simPrevT = getRobotTime();

    leftPrimary.setSelectedSensorPosition(driveSim.getLeftPositionMeters() / METERS_PER_TICK);
    rightPrimary.setSelectedSensorPosition(driveSim.getRightPositionMeters() / METERS_PER_TICK);
  }
}