package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveDriveSubsystem implements DriveSubsystem
{
  // Physical parameters
  public static final double ROBOT_TRACK_WIDTH = 0.672; // meters (30 in)
  public static final double ROBOT_LENGTH = 0.672; // meter 
  public static final double WHEEL_RADIUS = 0.0508; // meters (2 in)
  public static final double ENCODER_TICKS = 2048; // Ticks/rotation of Integrated encoder
  public static final double RADIANS_PER_TICK = 2 * Math.PI / ENCODER_TICKS;
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

  private static final int ENCODER_PORT = 0;

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
  private static final int PEAK_CURRENT_LIMIT = 6;
  private static final int CONTINUOUS_CURRENT_LIMIT = 5;
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
  private static final Translation2d leftFrontPosition = new Translation2d(-ROBOT_TRACK_WIDTH/2, ROBOT_LENGTH/2);
  private static final Translation2d leftBackPosition = new Translation2d(-ROBOT_TRACK_WIDTH/2, -ROBOT_LENGTH/2);
  private static final Translation2d rightFrontPosition = new Translation2d(ROBOT_TRACK_WIDTH/2, ROBOT_LENGTH/2);
  private static final Translation2d rightBackPosition = new Translation2d(ROBOT_TRACK_WIDTH/2, -ROBOT_LENGTH/2);
  private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontPosition, leftBackPosition, rightFrontPosition, rightBackPosition);

  //Might need extra parameters for rotator motors
  //Make like differential drive subsystem constructor
  public SwerveDriveSubsystem(TalonFXConfiguration mainConfig, TalonFXConfiguration rotatorConfig)
  {
    setFactoryMotorConfig();

    if(mainConfig != null)
    {
      leftFrontMotor.configAllSettings(mainConfig);
      leftBackMotor.configAllSettings(mainConfig);
      rightFrontMotor.configAllSettings(mainConfig);
      rightBackMotor.configAllSettings(mainConfig);
    }
    if(rotatorConfig != null){
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

    leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    leftBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    leftFrontRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    leftBackRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightBackRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightFrontRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    leftFrontMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rightFrontMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    leftBackMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rightBackMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    leftFrontRotatorMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    leftBackRotatorMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rightBackRotatorMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rightFrontRotatorMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    leftFrontMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rightFrontMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    leftBackMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rightBackMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

    leftFrontRotatorMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    leftBackRotatorMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rightBackRotatorMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rightFrontRotatorMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

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
  @Override
  public void setVelocities(ChassisSpeeds inputChassisSpeeds) {
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
    SmartDashboard.putNumber("S_Front_Left", modules[0].speedMetersPerSecond);
    SmartDashboard.putNumber("S_Back_Left", modules[1].speedMetersPerSecond);
    SmartDashboard.putNumber("S_Front_Right", modules[2].speedMetersPerSecond);
    SmartDashboard.putNumber("S_Back_Right", modules[3].speedMetersPerSecond);
    SmartDashboard.putNumber("O_Front_Left", modules[0].angle.getDegrees());
    SmartDashboard.putNumber("O_Back_Left", modules[1].angle.getDegrees());
    SmartDashboard.putNumber("O_Front_Right", modules[2].angle.getDegrees());
    SmartDashboard.putNumber("O_Back_Right", modules[3].angle.getDegrees());

    for(int i=0; i<4; i++)
      modules[i] = SwerveModuleState.optimize(modules[i], new Rotation2d(getRotatorEncoderPosition((i&1)==0, i>1)));

    leftFrontMotor.set(ControlMode.Velocity, modules[0].speedMetersPerSecond/(10 * METERS_PER_TICK));
    leftBackMotor.set(ControlMode.Velocity, modules[1].speedMetersPerSecond/(10 * METERS_PER_TICK));
    rightFrontMotor.set(ControlMode.Velocity, modules[2].speedMetersPerSecond/(10 * METERS_PER_TICK));
    rightBackMotor.set(ControlMode.Velocity, modules[3].speedMetersPerSecond/(10 * METERS_PER_TICK));

    leftFrontRotatorMotor.set(ControlMode.Position, modules[0].angle.getRadians()/(RADIANS_PER_TICK));
    leftBackRotatorMotor.set(ControlMode.Position, modules[1].angle.getRadians()/(RADIANS_PER_TICK));
    rightFrontRotatorMotor.set(ControlMode.Position, modules[2].angle.getRadians()/(RADIANS_PER_TICK));
    rightBackRotatorMotor.set(ControlMode.Position, modules[3].angle.getRadians()/(RADIANS_PER_TICK));
  } 

  public void setOutput(double omega, double XVelocity, double YVelocity)
  {
    setVelocities(new ChassisSpeeds(XVelocity, YVelocity, omega));
  }

  /**
   * Enables brake.
   */
  public void enableBrakes() {
    stop();
    stopRotators();
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
        SmartDashboard.putNumber("T_Back_Left", leftBackMotor.getSelectedSensorPosition(ENCODER_PORT)*METERS_PER_TICK);
        return leftBackMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        SmartDashboard.putNumber("T_Front_Left", leftFrontMotor.getSelectedSensorPosition(ENCODER_PORT)*METERS_PER_TICK);
        return leftFrontMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
    }
    else
    {
      if (useBack){
        SmartDashboard.putNumber("_Back_Right", rightBackMotor.getSelectedSensorPosition(ENCODER_PORT)*METERS_PER_TICK);
        return rightBackMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        SmartDashboard.putNumber("T_Front_Right", rightFrontMotor.getSelectedSensorPosition(ENCODER_PORT)*METERS_PER_TICK);
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
        SmartDashboard.putNumber("T_Rot_Back_Left", leftBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT)*360/ENCODER_TICKS);
        return leftBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        SmartDashboard.putNumber("T_Rot_Front_Left", leftFrontRotatorMotor.getSelectedSensorPosition(ENCODER_PORT)*360/ENCODER_TICKS);
        return leftFrontRotatorMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
    }
    else
    {
      if (useBack){
        SmartDashboard.putNumber("T_Rot_Back_Right", rightBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT)*360/ENCODER_TICKS);
        return rightBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        SmartDashboard.putNumber("T_Rot_Front_Right", rightFrontRotatorMotor.getSelectedSensorPosition(ENCODER_PORT)*360/ENCODER_TICKS);
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
  public double getRotatorEncoderPosition(boolean useLeft, boolean useBack) {
    return getRotatorEncoderCount(useLeft, useBack) * RADIANS_PER_TICK;
  }

  /**
   * Gets FPGA time from robot and converts it to seconds.
   * 
   * @return FPGA time in seconds.
   */
  public double getRobotTime() {

    return RobotController.getFPGATime() / 1e6;

  }
}
