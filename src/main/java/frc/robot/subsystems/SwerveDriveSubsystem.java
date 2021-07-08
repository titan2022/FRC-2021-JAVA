package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveDriveSubsystem implements DriveSubsystem
{
  // Physical parameters
  public static final double ROBOT_TRACK_WIDTH = Units.inchesToMeters(23.5); // 0.672; // meters (30 in)
  public static final double ROBOT_LENGTH = Units.inchesToMeters(23.5); // 0.672; // meter 
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2); // 0.0508; // meters (2 in)
  public static final double INTEGRATED_ENCODER_TICKS = 4096;//2048;
  public static final double CANCODER_TICKS = 4096;
  public static final double RADIANS_PER_TICK = 2 * Math.PI / CANCODER_TICKS;
  public static final double GEAR_RATIO = 6.86;
  public static final double METERS_PER_TICKS = WHEEL_RADIUS * 2 * Math.PI / INTEGRATED_ENCODER_TICKS / GEAR_RATIO;

  // Rotator Encoder Offsets
  private static final double LEFT_FRONT_ENCODER_DEGREES_OFFSET = 74.88*0;//77;  852
  private static final double LEFT_BACK_ENCODER_DEGREES_OFFSET = 70.22*0;//68.4;  799
  private static final double RIGHT_FRONT_ENCODER_DEGREES_OFFSET = -151.17*0;//-130;  -1720
  private static final double RIGHT_BACK_ENCODER_DEGREES_OFFSET = 18.46*0;//16;  210

  // Deadbands
  private static final double WHEEL_DEADBAND = 0.01;
  private static final double ROTATOR_DEADBAND = 0.001;
    
  // Port numbers to be added later
  private static final int LEFT_FRONT_MOTOR_PORT = 4;
  private static final int LEFT_BACK_MOTOR_PORT = 2;
  private static final int RIGHT_FRONT_MOTOR_PORT = 3;
  private static final int RIGHT_BACK_MOTOR_PORT = 7;
  private static final int LEFT_FRONT_MOTOR_ROTATOR_PORT = 6;
  private static final int LEFT_BACK_MOTOR_ROTATOR_PORT = 5;
  private static final int RIGHT_FRONT_MOTOR_ROTATOR_PORT = 1;
  private static final int RIGHT_BACK_MOTOR_ROTATOR_PORT = 0;

  private static final int LEFT_FRONT_ENCODER_ROTATOR_PORT = 10; // TODO: Get CANCoder ports
  private static final int LEFT_BACK_ENCODER_ROTATOR_PORT = 11;
  private static final int RIGHT_FRONT_ENCODER_ROTATOR_PORT = 9;
  private static final int RIGHT_BACK_ENCODER_ROTATOR_PORT = 8;

  private static final int ENCODER_PORT = 0;

  // Motor and sensor inversions
  private static final boolean LEFT_FRONT_MOTOR_INVERTED = false;
  private static final boolean LEFT_BACK_MOTOR_INVERTED = false;
  private static final boolean LEFT_FRONT_MOTOR_ROTATOR_INVERTED = false;
  private static final boolean LEFT_BACK_MOTOR_ROTATOR_INVERTED = false;
  private static final boolean RIGHT_FRONT_MOTOR_INVERTED = false;
  private static final boolean RIGHT_BACK_MOTOR_INVERTED = false;
  private static final boolean RIGHT_FRONT_MOTOR_ROTATOR_INVERTED = false;
  private static final boolean RIGHT_BACK_MOTOR_ROTATOR_INVERTED = false;
 
  private static final boolean LEFT_FRONT_MOTOR_SENSOR_PHASE = false; // TODO: Remove if sensor phase is correct
  private static final boolean RIGHT_FRONT_MOTOR_SENSOR_PHASE = false;
  private static final boolean LEFT_BACK_MOTOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_BACK_MOTOR_SENSOR_PHASE = false;
  private static final boolean LEFT_FRONT_MOTOR_ROTATOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_FRONT_MOTOR_ROTATOR_SENSOR_PHASE = false;
  private static final boolean LEFT_BACK_MOTOR_ROTATOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_BACK_MOTOR_ROTATOR_SENSOR_PHASE = false;

  // Physical limits of motors that create translational motion
  private static final double MAX_WHEEL_SPEED = .25; // meters/sec
  private static final int PEAK_CURRENT_LIMIT = 6;
  private static final int CONTINUOUS_CURRENT_LIMIT = 5;
  private static final StatorCurrentLimitConfiguration statorCurrentLimit = new StatorCurrentLimitConfiguration(true,
      PEAK_CURRENT_LIMIT, 0, 0);
  private static final SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true,
      CONTINUOUS_CURRENT_LIMIT, 0, 0);

  // Physical limits of motors that rotate the wheel. Use radians.
  
  // Physical Hardware
  private static final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(LEFT_FRONT_MOTOR_PORT)
    , leftBackMotor = new WPI_TalonFX(LEFT_BACK_MOTOR_PORT)
    , rightFrontMotor = new WPI_TalonFX(RIGHT_FRONT_MOTOR_PORT)
    , rightBackMotor = new WPI_TalonFX(RIGHT_BACK_MOTOR_PORT)
    , leftFrontRotatorMotor = new WPI_TalonFX(LEFT_FRONT_MOTOR_ROTATOR_PORT)
    , leftBackRotatorMotor = new WPI_TalonFX(LEFT_BACK_MOTOR_ROTATOR_PORT)
    , rightFrontRotatorMotor = new WPI_TalonFX(RIGHT_FRONT_MOTOR_ROTATOR_PORT)
    , rightBackRotatorMotor = new WPI_TalonFX(RIGHT_BACK_MOTOR_ROTATOR_PORT);
  
    private static final CANCoder leftFrontRotatorEncoder = new CANCoder(LEFT_FRONT_ENCODER_ROTATOR_PORT)
      , leftBackRotatorEncoder = new CANCoder(LEFT_BACK_ENCODER_ROTATOR_PORT)
      , rightFrontRotatorEncoder = new CANCoder(RIGHT_FRONT_ENCODER_ROTATOR_PORT)
      , rightBackRotatorEncoder = new CANCoder(RIGHT_BACK_ENCODER_ROTATOR_PORT);

  // PID for rotators
  // One slot and one PID_IDX
  private static final int ROTATOR_SLOT_IDX = 0;
  private static final int ROTATOR_PID_IDX = 0;

  // PID for main motors
  private static final int MAIN_MOTOR_SLOT_IDX = 0;
  private static final int MAIN_MOTOR_PID_IDX = 0;

  // Kinematics
  // Positions describe the position of each wheel relative to the center of the robot
  private static final Translation2d leftFrontPosition = new Translation2d(-ROBOT_TRACK_WIDTH/2, ROBOT_LENGTH/2);
  private static final Translation2d leftBackPosition = new Translation2d(-ROBOT_TRACK_WIDTH/2, -ROBOT_LENGTH/2);
  private static final Translation2d rightFrontPosition = new Translation2d(ROBOT_TRACK_WIDTH/2, ROBOT_LENGTH/2);
  private static final Translation2d rightBackPosition = new Translation2d(ROBOT_TRACK_WIDTH/2, -ROBOT_LENGTH/2);
  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontPosition, leftBackPosition, rightFrontPosition, rightBackPosition);

  /**
   * Creates the swerve drive subsystem
   * @param mainConfig Requires PID configuration in slot 0
   * @param rotatorConfig Requires PID configuration in slot 0
   */
  public SwerveDriveSubsystem(TalonFXConfiguration mainConfig, TalonFXConfiguration rotatorConfig)
  {
    setFactoryMotorConfig();

    mainConfig = mainConfig == null ? new TalonFXConfiguration() : mainConfig;
    rotatorConfig = rotatorConfig == null ? new TalonFXConfiguration() : rotatorConfig;

    mainConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    
    rotatorConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    rotatorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

    mainConfig.statorCurrLimit = statorCurrentLimit;
    mainConfig.supplyCurrLimit = supplyCurrentLimit;
    rotatorConfig.statorCurrLimit = statorCurrentLimit;
    rotatorConfig.supplyCurrLimit = supplyCurrentLimit;

    rotatorConfig.neutralDeadband = ROTATOR_DEADBAND;
    mainConfig.neutralDeadband = WHEEL_DEADBAND;

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
    
    leftFrontRotatorMotor.configRemoteFeedbackFilter(leftFrontRotatorEncoder, 0);
    leftBackRotatorMotor.configRemoteFeedbackFilter(leftBackRotatorEncoder, 0);
    rightFrontRotatorMotor.configRemoteFeedbackFilter(rightFrontRotatorEncoder, 0);
    rightBackRotatorMotor.configRemoteFeedbackFilter(rightBackRotatorEncoder, 0);

    leftFrontRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    leftBackRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    rightBackRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    rightFrontRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);

    leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    leftBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    /* Moved to config object
    leftFrontMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    leftBackMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    rightFrontMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    rightBackMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);*/

    // CANCoder Configuration
    /* Moved to config object
    leftFrontRotatorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    leftBackRotatorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rightFrontRotatorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rightBackRotatorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    */

    leftFrontRotatorEncoder.configMagnetOffset(LEFT_FRONT_ENCODER_DEGREES_OFFSET);
    leftBackRotatorEncoder.configMagnetOffset(LEFT_BACK_ENCODER_DEGREES_OFFSET);
    rightFrontRotatorEncoder.configMagnetOffset(RIGHT_FRONT_ENCODER_DEGREES_OFFSET);
    rightBackRotatorEncoder.configMagnetOffset(RIGHT_BACK_ENCODER_DEGREES_OFFSET);

    /* Moved to config object
    leftFrontRotatorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    leftBackRotatorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rightFrontRotatorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rightBackRotatorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    */

    // TalonFX configuration
    rightFrontMotor.setInverted(RIGHT_FRONT_MOTOR_INVERTED);
    rightBackMotor.setInverted(RIGHT_BACK_MOTOR_INVERTED);
    rightFrontRotatorMotor.setInverted(RIGHT_FRONT_MOTOR_ROTATOR_INVERTED);
    rightBackRotatorMotor.setInverted(RIGHT_BACK_MOTOR_ROTATOR_INVERTED);

    leftFrontMotor.setInverted(LEFT_FRONT_MOTOR_INVERTED);
    leftBackMotor.setInverted(LEFT_BACK_MOTOR_INVERTED);
    leftFrontRotatorMotor.setInverted(LEFT_FRONT_MOTOR_ROTATOR_INVERTED);
    leftBackRotatorMotor.setInverted(LEFT_BACK_MOTOR_ROTATOR_INVERTED);

    // Sets the direction that the talon will turn on the green LED when going
    // 'forward'.
    leftFrontMotor.setSensorPhase(LEFT_FRONT_MOTOR_SENSOR_PHASE);
    rightFrontMotor.setSensorPhase(RIGHT_FRONT_MOTOR_SENSOR_PHASE);
    leftBackMotor.setSensorPhase(LEFT_BACK_MOTOR_SENSOR_PHASE);
    rightBackMotor.setSensorPhase(RIGHT_BACK_MOTOR_SENSOR_PHASE);
    leftFrontRotatorMotor.setSensorPhase(LEFT_FRONT_MOTOR_ROTATOR_SENSOR_PHASE);
    rightFrontRotatorMotor.setSensorPhase(RIGHT_FRONT_MOTOR_ROTATOR_SENSOR_PHASE);
    leftBackRotatorMotor.setSensorPhase(LEFT_BACK_MOTOR_ROTATOR_SENSOR_PHASE);
    rightBackRotatorMotor.setSensorPhase(RIGHT_BACK_MOTOR_ROTATOR_SENSOR_PHASE);

    // Current limits
    /* Moved to config object
    leftFrontMotor.configStatorCurrentLimit(statorCurrentLimit);
    leftFrontMotor.configSupplyCurrentLimit(supplyCurrentLimit);
    rightFrontMotor.configStatorCurrentLimit(statorCurrentLimit);
    rightFrontMotor.configSupplyCurrentLimit(supplyCurrentLimit);
    leftBackMotor.configStatorCurrentLimit(statorCurrentLimit);
    leftBackMotor.configSupplyCurrentLimit(supplyCurrentLimit);
    rightBackMotor.configStatorCurrentLimit(statorCurrentLimit);
    rightBackMotor.configSupplyCurrentLimit(supplyCurrentLimit);

    leftFrontRotatorMotor.configStatorCurrentLimit(statorCurrentLimit);
    leftFrontRotatorMotor.configSupplyCurrentLimit(supplyCurrentLimit);
    rightFrontRotatorMotor.configStatorCurrentLimit(statorCurrentLimit);
    rightFrontRotatorMotor.configSupplyCurrentLimit(supplyCurrentLimit);
    leftBackRotatorMotor.configStatorCurrentLimit(statorCurrentLimit);
    leftBackRotatorMotor.configSupplyCurrentLimit(supplyCurrentLimit);
    rightBackRotatorMotor.configStatorCurrentLimit(statorCurrentLimit);
    rightBackRotatorMotor.configSupplyCurrentLimit(supplyCurrentLimit);
    */

    //neutral deadbands
    /* Moved to config object
    leftFrontRotatorMotor.configNeutralDeadband(ROTATOR_DEADBAND);
    rightFrontRotatorMotor.configNeutralDeadband(ROTATOR_DEADBAND);
    leftBackRotatorMotor.configNeutralDeadband(ROTATOR_DEADBAND);
    rightBackRotatorMotor.configNeutralDeadband(ROTATOR_DEADBAND);

    leftFrontMotor.configNeutralDeadband(WHEEL_DEADBAND);
    leftBackMotor.configNeutralDeadband(WHEEL_DEADBAND);
    rightFrontMotor.configNeutralDeadband(WHEEL_DEADBAND);
    rightBackMotor.configNeutralDeadband(WHEEL_DEADBAND);
    */

    /* TODO: Implement logic in the future to deal with motor faults to add robustness
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
  }

  public SwerveDriveSubsystem() {
    this(null, null);
  }

  private void setFactoryMotorConfig() {
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
    return MAX_WHEEL_SPEED;
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
    SwerveModuleState[] modules = kinematics.toSwerveModuleStates(inputChassisSpeeds);
    SmartDashboard.putNumber("State FL vel", modules[0].speedMetersPerSecond);
    SmartDashboard.putNumber("State BL vel", modules[1].speedMetersPerSecond);
    SmartDashboard.putNumber("State FR vel", modules[2].speedMetersPerSecond);
    SmartDashboard.putNumber("State BR vel", modules[3].speedMetersPerSecond);
    SmartDashboard.putNumber("State FL deg", modules[0].angle.getDegrees());
    SmartDashboard.putNumber("State BL deg", modules[1].angle.getDegrees());
    SmartDashboard.putNumber("State FR deg", modules[2].angle.getDegrees());
    SmartDashboard.putNumber("State BR deg", modules[3].angle.getDegrees());

    SwerveDriveKinematics.normalizeWheelSpeeds(modules, MAX_WHEEL_SPEED);
    SmartDashboard.putNumber("Norm FL vel", modules[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm BL vel", modules[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm FR vel", modules[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm BR vel", modules[3].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm FL deg", modules[0].angle.getDegrees());
    SmartDashboard.putNumber("Norm BL deg", modules[1].angle.getDegrees());
    SmartDashboard.putNumber("Norm FR deg", modules[2].angle.getDegrees());
    SmartDashboard.putNumber("Norm BR deg", modules[3].angle.getDegrees());
    SmartDashboard.putNumber("Norm FL tgt", modules[0].angle.getRadians() / RADIANS_PER_TICK);
    SmartDashboard.putNumber("Norm BL tgt", modules[1].angle.getRadians() / RADIANS_PER_TICK);
    SmartDashboard.putNumber("Norm FR tgt", modules[2].angle.getRadians() / RADIANS_PER_TICK);
    SmartDashboard.putNumber("Norm BR tgt", modules[3].angle.getRadians() / RADIANS_PER_TICK);
    
    // for(int i=0; i<4; i++)
    //   modules[i] = SwerveModuleState.optimize(modules[i], new Rotation2d(getRotatorEncoderPosition((i&1)==0, i>1)));

    leftFrontMotor.set(ControlMode.Velocity, modules[0].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    leftBackMotor.set(ControlMode.Velocity, modules[1].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    rightFrontMotor.set(ControlMode.Velocity, modules[2].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    rightBackMotor.set(ControlMode.Velocity, modules[3].speedMetersPerSecond/(10 * METERS_PER_TICKS));

    
    leftFrontRotatorMotor.set(ControlMode.Position, modules[0].angle.getRadians() / RADIANS_PER_TICK + 908);
    leftBackRotatorMotor.set(ControlMode.Position, modules[1].angle.getRadians() / RADIANS_PER_TICK + 773);
    rightFrontRotatorMotor.set(ControlMode.Position, modules[2].angle.getRadians() / RADIANS_PER_TICK - 1783);
    rightBackRotatorMotor.set(ControlMode.Position, modules[3].angle.getRadians() / RADIANS_PER_TICK + 231);
    

    getSwerveModuleStates();
  } 

  public void setOutput(double omega, double XVelocity, double YVelocity)
  {
    setVelocities(new ChassisSpeeds(XVelocity, YVelocity, omega));
  }

  // TODO: Fix all the brake logic and semantics because disabling brakes into coast mode is not about disabling brakes.

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
   * 
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getEncoderCount(boolean useLeft, boolean useBack)
  {
    if (useLeft)
    {
      if (useBack){
        SmartDashboard.putNumber("read BL m", leftBackMotor.getSelectedSensorPosition(ENCODER_PORT)*METERS_PER_TICKS);
        return leftBackMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        SmartDashboard.putNumber("read FL m", leftFrontMotor.getSelectedSensorPosition(ENCODER_PORT)*METERS_PER_TICKS);
        return leftFrontMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
    }
    else
    {
      if (useBack){
        SmartDashboard.putNumber("read BR m", rightBackMotor.getSelectedSensorPosition(ENCODER_PORT)*METERS_PER_TICKS);
        return rightBackMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
      else{
        SmartDashboard.putNumber("read FR m", rightFrontMotor.getSelectedSensorPosition(ENCODER_PORT)*METERS_PER_TICKS);
        return rightFrontMotor.getSelectedSensorPosition(ENCODER_PORT);
      }
    }
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * 
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getRotatorEncoderCount(boolean useLeft, boolean useBack)
  {
    if (useLeft)
    {
      if (useBack){
        SmartDashboard.putNumber("raw BL rot", leftBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT));
        return leftBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT) * RADIANS_PER_TICK;
      }
      else{
        SmartDashboard.putNumber("raw FL rot", leftFrontRotatorMotor.getSelectedSensorPosition(ENCODER_PORT));
        return leftFrontRotatorMotor.getSelectedSensorPosition(ENCODER_PORT) * RADIANS_PER_TICK;
      }
    }
    else
    {
      if (useBack){
        SmartDashboard.putNumber("raw BR rot", rightBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT));
        return rightBackRotatorMotor.getSelectedSensorPosition(ENCODER_PORT) * RADIANS_PER_TICK;
      }
      else{
        SmartDashboard.putNumber("raw FR rot", rightFrontRotatorMotor.getSelectedSensorPosition(ENCODER_PORT));
        return rightFrontRotatorMotor.getSelectedSensorPosition(ENCODER_PORT) * RADIANS_PER_TICK;
      }
    }
  }

  /**
   * Gets the amount of rotation from a primary motor.
   * 
   * @param useLeft - Whether to use the left primary motor.
   * @return Angle of rotator motor in radians
   */
  public double getRotatorEncoderPosition(boolean useLeft, boolean useBack) {
    return getRotatorEncoderCount(useLeft, useBack);
  }

  public double getEncoderVelocity(boolean useLeft, boolean useBack)
  {
    if (useLeft)
    {
      if (useBack){
        SmartDashboard.putNumber("read BL m/s", leftBackMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICKS * 10);
        return leftBackMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICKS * 10;
      }
      else{
        SmartDashboard.putNumber("read FL m/s", leftFrontMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICKS * 10);
        return leftFrontMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICKS * 10;
      }
    }
    else
    {
      if (useBack){
        SmartDashboard.putNumber("read BR m/s", rightBackMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICKS * 10);
        return rightBackMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICKS * 10;
      }
      else{
        SmartDashboard.putNumber("read FR m/s", rightFrontMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICKS * 10);
        return rightFrontMotor.getSelectedSensorVelocity(ENCODER_PORT) * METERS_PER_TICKS * 10;
      }
    }
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState leftBack = new SwerveModuleState(getEncoderVelocity(true, true),
        new Rotation2d(getRotatorEncoderPosition(true, true)));

    SwerveModuleState leftFront = new SwerveModuleState(getEncoderVelocity(true, false),
        new Rotation2d(getRotatorEncoderPosition(true, false)));

    SwerveModuleState rightBack = new SwerveModuleState(getEncoderVelocity(false, true),
        new Rotation2d(getRotatorEncoderPosition(false, true)));

    SwerveModuleState rightFront = new SwerveModuleState(getEncoderVelocity(false, false),
        new Rotation2d(getRotatorEncoderPosition(false, false)));
    
    return new SwerveModuleState[]{leftFront, leftBack, rightFront, rightBack};
  }

  @Override
  public void periodic() {
    getEncoderVelocity(false, false);
    getEncoderVelocity(false, true);
    getEncoderVelocity(true, false);
    getEncoderVelocity(true, true);
  }
}
