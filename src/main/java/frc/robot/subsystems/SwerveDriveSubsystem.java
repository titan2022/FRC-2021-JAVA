package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase
{
  // Physical parameters
  public static final double ROBOT_TRACK_WIDTH = 0.672; // meters (30 in)
  public static final double ROBOT_LENGTH = 0.672; // meter 
  public static final double WHEEL_RADIUS = 0.0508; // meters (2 in)
  public static final double ENCODER_TICKS = 4096; // Ticks/rotation of CTREMagEncoder
  public static final double METERS_PER_TICK = WHEEL_RADIUS * 2 * Math.PI / ENCODER_TICKS;
  
    
  // Port numbers to be added later
  private static final int LEFT_FRONT_MOTOR_PORT = 2;
  private static final int LEFT_BACK_MOTOR_PORT = 7;
  private static final int RIGHT_FRONT_MOTOR_PORT = 4;
  private static final int RIGHT_BACK_MOTOR_PORT = 3;
  private static final int LEFT_FRONT_MOTOR_ROTATOR_PORT = 5;
  private static final int LEFT_BACK_MOTOR_ROTATOR_PORT = 0;
  private static final int RIGHT_FRONT_MOTOR_ROTATOR_PORT = 6;
  private static final int RIGHT_BACK_MOTOR_ROTATOR_PORT = 1;

  private static final int ENCODER_PORT = 1;

  // Motor and sensor inversions
  // TODO: Rename primary and secondary to front and back. Need inversion variable for ever single motor.
  private static final boolean LEFT_FRONT_MOTOR_INVERTED = false;
  private static final boolean LEFT_BACK_MOTOR_INVERTED = false;
  private static final boolean LEFT_FRONT_MOTOR_ROTATOR_INVERTED = false;
  private static final boolean LEFT_BACK_MOTOR_ROTATOR_INVERTED = false;
  private static final boolean RIGHT_FRONT_MOTOR_INVERTED = false;
  private static final boolean RIGHT_BACK_MOTOR_INVERTED = false;
  private static final boolean RIGHT_FRONT_MOTOR_ROTATOR_INVERTED = false;
  private static final boolean RIGHT_BACK_MOTOR_ROTATOR_INVERTED = false;

  // TODO: Need variables for all the sensor phases. Rename primary and secondary to front and back. 
  private static final boolean LEFT_FRONT_MOTOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_FRONT_MOTOR_SENSOR_PHASE = false;
  private static final boolean LEFT_BACK_MOTOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_BACK_MOTOR_SENSOR_PHASE = false;
  private static final boolean LEFT_FRONT_MOTOR_ROTATOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_FRONT_MOTOR_ROTATOR_SENSOR_PHASE = false;
  private static final boolean LEFT_BACK_MOTOR__ROTATOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_BACK_MOTOR_ROTATOR_SENSOR_PHASE = false;

  // Physical limits of motors that create translational motion
  private static final double MAX_SPEED = 10; // meters/sec
  private static final int PEAK_CURRENT_LIMIT = 60;
  private static final int CONTINUOUS_CURRENT_LIMIT = 50;
  private static final StatorCurrentLimitConfiguration statorCurrentLimit = new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0);
  private static final SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0);

  // Physical limits of motors that rotate the wheel. Change to radians.
  private static final double MAX_ROTATIONAL_SPEED = 10; // radians/sec
  
  // Physical and Simulated Hardware
  // These talon objects are also simulated
  private static final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(LEFT_FRONT_MOTOR_PORT)
    , leftBackMotor = new WPI_TalonFX(LEFT_BACK_MOTOR_PORT)
    , rightFrontMotor = new WPI_TalonFX(RIGHT_FRONT_MOTOR_PORT)
    , rightBackMotor = new WPI_TalonFX(RIGHT_BACK_MOTOR_PORT)
    , leftFrontRotatorMotor = new WPI_TalonFX(LEFT_FRONT_MOTOR_ROTATOR_PORT)
    , leftBackRotatorMotor = new WPI_TalonFX(LEFT_BACK_MOTOR_ROTATOR_PORT)
    , rightFrontRotatorMotor = new WPI_TalonFX(RIGHT_FRONT_MOTOR_ROTATOR_PORT)
    , rightBackRotatorMotor = new WPI_TalonFX(RIGHT_BACK_MOTOR_ROTATOR_PORT);

  // PID for rotators
  // One slot and one PID_IDX
  private static final int ROTATOR_SLOT_IDX = 0;
  private static final int ROTATOR_PID_IDX = 0;

  // PID for main motors
  private static final int MAIN_MOTOR_SLOT_IDX = 0;
  private static final int MAIN_MOTOR_PID_IDX = 0;

  //Kinematics
  //positions describe the position of each wheel relative to the center of the robot
  SwerveDriveKinematics kinematics;
  private static final Translation2d leftFrontPosition = new Translation2d(-ROBOT_TRACK_WIDTH/2, ROBOT_LENGTH/2);
  private static final Translation2d leftBackPosition = new Translation2d(-ROBOT_TRACK_WIDTH/2, -ROBOT_LENGTH/2);
  private static final Translation2d rightFrontPosition = new Translation2d(ROBOT_TRACK_WIDTH/2, ROBOT_LENGTH/2);
  private static final Translation2d rightBackPosition = new Translation2d(ROBOT_TRACK_WIDTH/2, -ROBOT_LENGTH/2);

  //Might need extra parameters for rotator motors
  //Make like differential drive subsystem constructor
  public SwerveDriveSubsystem(TalonFXConfiguration mainConfig, TalonFXConfiguration rotatorConfig)
  {
    setFactoryMotorConfig();

    if(mainConfig != null && rotatorConfig != null)
    {
    leftFrontMotor.configAllSettings(mainConfig);
    leftBackMotor.configAllSettings(mainConfig);
    rightFrontMotor.configAllSettings(mainConfig);
    rightBackMotor.configAllSettings(mainConfig);

    leftFrontRotatorMotor.configAllSettings(rotatorConfig);
    leftBackRotatorMotor.configAllSettings(rotatorConfig);
    rightFrontRotatorMotor.configAllSettings(rotatorConfig);
    rightBackRotatorMotor.configAllSettings(rotatorConfig);
    }

    rightFrontMotor.setInverted(RIGHT_FRONT_MOTOR_INVERTED);
    rightBackMotor.setInverted(RIGHT_BACK_MOTOR_INVERTED);
    rightFrontRotatorMotor.setInverted(RIGHT_FRONT_MOTOR_ROTATOR_INVERTED);
    rightBackRotatorMotor.setInverted(RIGHT_BACK_MOTOR_ROTATOR_INVERTED);

    leftFrontMotor.setInverted(LEFT_FRONT_MOTOR_INVERTED);
    leftBackMotor.setInverted(LEFT_BACK_MOTOR_INVERTED);
    leftFrontRotatorMotor.setInverted(LEFT_FRONT_MOTOR_ROTATOR_INVERTED);
    leftBackRotatorMotor.setInverted(LEFT_BACK_MOTOR_ROTATOR_INVERTED);

    // Sets the direction that the talon will turn on the green LED when going 'forward'.
    leftFrontMotor.setSensorPhase(LEFT_FRONT_MOTOR_SENSOR_PHASE);
    rightFrontMotor.setSensorPhase(RIGHT_FRONT_MOTOR_SENSOR_PHASE);
    leftBackMotor.setSensorPhase(LEFT_BACK_MOTOR_SENSOR_PHASE);
    rightBackMotor.setSensorPhase(RIGHT_BACK_MOTOR_SENSOR_PHASE);
    leftFrontRotatorMotor.setSensorPhase(LEFT_FRONT_MOTOR_ROTATOR_SENSOR_PHASE);
    rightFrontRotatorMotor.setSensorPhase(RIGHT_FRONT_MOTOR_ROTATOR_SENSOR_PHASE);
    leftBackRotatorMotor.setSensorPhase(LEFT_BACK_MOTOR__ROTATOR_SENSOR_PHASE);
    rightBackRotatorMotor.setSensorPhase(RIGHT_BACK_MOTOR_ROTATOR_SENSOR_PHASE);

    // Current limits in amps
    // TODO: Find equivalent method names for FX stuff
    leftFrontMotor.configStatorCurrentLimit(statorCurrentLimit);
    leftFrontMotor.configSupplyCurrentLimit(supplyCurrentLimit);

    rightFrontMotor.configStatorCurrentLimit(statorCurrentLimit);
    rightFrontMotor.configSupplyCurrentLimit(supplyCurrentLimit);

    leftFrontRotatorMotor.configStatorCurrentLimit(statorCurrentLimit);
    leftFrontRotatorMotor.configSupplyCurrentLimit(supplyCurrentLimit);

    leftBackRotatorMotor.configStatorCurrentLimit(statorCurrentLimit);
    leftBackRotatorMotor.configSupplyCurrentLimit(supplyCurrentLimit);

    leftBackMotor.configStatorCurrentLimit(statorCurrentLimit);
    leftBackMotor.configSupplyCurrentLimit(supplyCurrentLimit);

    rightFrontRotatorMotor.configStatorCurrentLimit(statorCurrentLimit);
    rightFrontRotatorMotor.configSupplyCurrentLimit(supplyCurrentLimit);

    rightBackMotor.configStatorCurrentLimit(statorCurrentLimit);
    rightBackMotor.configSupplyCurrentLimit(supplyCurrentLimit);

    rightBackRotatorMotor.configStatorCurrentLimit(statorCurrentLimit);
    rightBackRotatorMotor.configSupplyCurrentLimit(supplyCurrentLimit);

    // Might need more for rotator motors

    /* TODO: Deal with motor controller faults once a physical robot is available for testing
    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();
    */

    leftFrontRotatorMotor.selectProfileSlot(ROTATOR_SLOT_IDX, ROTATOR_PID_IDX);
    rightFrontRotatorMotor.selectProfileSlot(ROTATOR_SLOT_IDX, ROTATOR_PID_IDX);
    leftBackRotatorMotor.selectProfileSlot(ROTATOR_SLOT_IDX, ROTATOR_PID_IDX);
    rightBackRotatorMotor.selectProfileSlot(ROTATOR_SLOT_IDX, ROTATOR_PID_IDX);

    leftFrontMotor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, MAIN_MOTOR_PID_IDX);
    rightFrontMotor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, MAIN_MOTOR_PID_IDX);
    leftBackMotor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, MAIN_MOTOR_PID_IDX);
    rightBackMotor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, MAIN_MOTOR_PID_IDX);

    leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    leftBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    rightBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    leftFrontRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    leftBackRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    rightBackRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    rightFrontRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    //Kinematics
    //order is leftfront, leftback, rightfront, rightback
    kinematics = new SwerveDriveKinematics(leftFrontPosition, leftBackPosition, rightFrontPosition, rightBackPosition);

    //neutral deadbands
    leftFrontRotatorMotor.configNeutralDeadband(0.199413);
    rightFrontRotatorMotor.configNeutralDeadband(0.199413);
    leftBackRotatorMotor.configNeutralDeadband(0.199413);
    rightBackRotatorMotor.configNeutralDeadband(0.199413);

    leftFrontMotor.configNeutralDeadband(0.199413);
    leftBackMotor.configNeutralDeadband(0.199413);
    rightFrontMotor.configNeutralDeadband(0.199413);
    rightBackMotor.configNeutralDeadband(0.199413);
  }

  public SwerveDriveSubsystem()
  {
    this(null, null);
  }

  private void setFactoryMotorConfig()
  {
    leftFrontMotor.configFactoryDefault();
    leftBackMotor.configFactoryDefault();
    rightFrontMotor.configFactoryDefault();
    rightBackMotor.configFactoryDefault();
    leftFrontRotatorMotor.configFactoryDefault();
    leftBackRotatorMotor.configFactoryDefault();
    rightFrontRotatorMotor.configFactoryDefault();
    rightBackRotatorMotor.configFactoryDefault();
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
   * TODO: Swerve drive modules
   * 
   * @param mode             a ControlMode enum
   * @param leftOutputValue  left side output value for ControlMode
   * @param rightOutputValue right side output value for ControlMode
   */
  public void setOutput(ChassisSpeeds inputChassisSpeeds) {
    // TODO: if check the current usage from Power Subsystem to restrict overcurrent
    // TODO: make this not fucking suck lol
    // TODO: make the controlmode output correspond to actual wheel velocity in m/s

    // double A, B, C, D;

    // A=XVelocity-(omega * (ROBOT_LENGTH/2));
    // B=XVelocity+(omega * (ROBOT_LENGTH/2));
    // C=XVelocity-(omega * (ROBOT_TRACK_WIDTH/2));
    // D=XVelocity+(omega * (ROBOT_TRACK_WIDTH/2));

    // leftPrimaryOutput = Math.sqrt((B*B)+(D*D));
    // rightPrimaryOutput = Math.sqrt((B*B)+(C*C));
    // leftSecondaryOutput = Math.sqrt((A*A)+(D*D));
    // rightSecondaryOutput = Math.sqrt((A*A)+(C*C));

    // //outputs angles, 0 is dead ahead, values go from -180 to 180
    // leftPrimaryRotatorSetpoint = Math.atan2(B, D)*(ENCODER_TICKS/(2*Math.PI));
    // rightPrimaryRotatorSetpoint = Math.atan2(B, C)*(ENCODER_TICKS/(2*Math.PI));
    // leftSecondaryRotatorSetpoint = Math.atan2(A, D)*(ENCODER_TICKS/(2*Math.PI));
    // rightSecondaryRotatorSetpoint = Math.atan2(A, C)*(ENCODER_TICKS/(2*Math.PI));

    SwerveModuleState[] modules = kinematics.toSwerveModuleStates(inputChassisSpeeds);


    leftFrontMotor.set(ControlMode.Velocity, modules[0].speedMetersPerSecond/METERS_PER_TICK);
    leftBackMotor.set(ControlMode.Velocity, modules[1].speedMetersPerSecond/METERS_PER_TICK);
    rightFrontMotor.set(ControlMode.Velocity, modules[2].speedMetersPerSecond/METERS_PER_TICK);
    rightBackMotor.set(ControlMode.Velocity, modules[3].speedMetersPerSecond/METERS_PER_TICK);

    leftFrontRotatorMotor.set(ControlMode.Position, modules[0].angle.getDegrees()*(ENCODER_TICKS/360));
    leftBackRotatorMotor.set(ControlMode.Position, modules[1].angle.getDegrees()*(ENCODER_TICKS/360));
    rightFrontRotatorMotor.set(ControlMode.Position, modules[2].angle.getDegrees()*(ENCODER_TICKS/360));
    rightBackRotatorMotor.set(ControlMode.Position, modules[3].angle.getDegrees()*(ENCODER_TICKS/360));
  } 

  public void setOutput(double omega, double XVelocity, double YVelocity)
  {
    setOutput(new ChassisSpeeds(XVelocity, YVelocity, omega));
  }

  /**
   * Enables brake.
   */
  public void enableBrakes() {
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftBackMotor.setNeutralMode(NeutralMode.Brake);
    rightBackMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Disables brake.
   */
  public void disableBrakes() {
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Enables brake for rotator motors.
   */
  public void enableRotatorBrakes() {
    leftFrontRotatorMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontRotatorMotor.setNeutralMode(NeutralMode.Brake);
    leftBackRotatorMotor.setNeutralMode(NeutralMode.Brake);
    rightBackRotatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Disables brake for rotator motors.
   */
  public void disableRotatorBrakes() {
    leftFrontRotatorMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontRotatorMotor.setNeutralMode(NeutralMode.Coast);
    leftBackRotatorMotor.setNeutralMode(NeutralMode.Coast);
    rightBackRotatorMotor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Stops the motors.
   */
  public void stop() {
    leftFrontMotor.set(ControlMode.PercentOutput, 0);
    rightFrontMotor.set(ControlMode.PercentOutput, 0);
    leftBackMotor.set(ControlMode.PercentOutput, 0);
    rightBackMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Stops the rotator motors.
   */
  public void stopRotators() {
    leftFrontRotatorMotor.set(ControlMode.PercentOutput, 0);
    rightFrontRotatorMotor.set(ControlMode.PercentOutput, 0);
    leftBackRotatorMotor.set(ControlMode.PercentOutput, 0);
    rightBackRotatorMotor.set(ControlMode.PercentOutput, 0);
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getEncoderCount(boolean useLeft, boolean useBack)
  {
    if (useLeft)
    {
      if (useBack){
        return leftBackMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        return leftFrontMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
    }
    else
    {
      if (useBack){
        return rightBackMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        return rightFrontMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
    }
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getRotatorEncoderCount(boolean useLeft, boolean useBack)
  {
    if (useLeft)
    {
      if (useBack){
        return leftBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        return leftFrontRotatorMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
    }
    else
    {
      if (useBack){
        return rightBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        return rightFrontRotatorMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
    }
  }

  /**
   * Gets the distance from a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Distance from specified primary motor.
   */
  public double getEncoderDist(boolean useLeft, boolean useBack) {
    return getEncoderCount(useLeft, useBack) * METERS_PER_TICK;
  }

  /**
   * Gets the amount of rotation from a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Rotation of a specified primary motor.
   */
  public double getRotatorEncoderDist(boolean useLeft, boolean useBack) {
    return getRotatorEncoderCount(useLeft, useBack) * METERS_PER_TICK / WHEEL_RADIUS;
  }

  /**
   * Gets current velocity of a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Current velocity of a specified primary motor.
   */
  public double getEncoderVelocity(boolean useLeft) {
    if (useLeft)
    {
      return leftFrontMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK;
    }
    else
    {
      return rightFrontMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK;
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
      return leftFrontMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK / WHEEL_RADIUS;
    }
    else
    {
      return rightFrontMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICK / WHEEL_RADIUS;
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
