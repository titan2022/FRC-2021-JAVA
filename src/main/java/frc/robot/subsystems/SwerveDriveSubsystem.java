package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase
{
  // Physical parameters
  public static final double ROBOT_TRACK_WIDTH = 26.75/39.37; // meter
  public static final double ROBOT_LENGTH = 0.5; // meter 
  public static final double WHEEL_RADIUS = 6/39.37; // meters
  public static final double ENCODER_TICKS = 4096; // Ticks/rotation of CTREMagEncoder
  public static final double METERS_PER_TICK = WHEEL_RADIUS * 2 * Math.PI / ENCODER_TICKS;
  
    
  // Port numbers to be added later
  private static final int LEFT_FRONT_MOTOR_PORT = 1;
  private static final int LEFT_BACK_MOTOR_PORT = 2;
  private static final int RIGHT_FRONT_MOTOR_PORT = 3;
  private static final int RIGHT_BACK_MOTOR_PORT = 4;
  private static final int LEFT_FRONT_MOTOR_ROTATOR_PORT = 5;
  private static final int LEFT_BACK_MOTOR_ROTATOR_PORT = 6;
  private static final int RIGHT_FRONT_MOTOR_ROTATOR_PORT = 7;
  private static final int RIGHT_BACK_MOTOR_ROTATOR_PORT = 8;

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

  // Physical limits of motors that rotate the wheel. Change to radians.
  private static final double MAX_ROTATIONAL_SPEED = 10; // radians/sec
  
  // Physical and Simulated Hardware
  // These talon objects are also simulated
  private static final WPI_TalonFX leftPrimary = new WPI_TalonFX(LEFT_FRONT_MOTOR_PORT)
    , leftSecondary = new WPI_TalonFX(LEFT_BACK_MOTOR_PORT)
    , rightPrimary = new WPI_TalonFX(RIGHT_FRONT_MOTOR_PORT)
    , rightSecondary = new WPI_TalonFX(RIGHT_BACK_MOTOR_PORT)
    , leftPrimaryRotator = new WPI_TalonFX(LEFT_FRONT_MOTOR_ROTATOR_PORT)
    , leftSecondaryRotator = new WPI_TalonFX(LEFT_BACK_MOTOR_ROTATOR_PORT)
    , rightPrimaryRotator = new WPI_TalonFX(RIGHT_FRONT_MOTOR_ROTATOR_PORT)
    , rightSecondaryRotator = new WPI_TalonFX(RIGHT_BACK_MOTOR_ROTATOR_PORT);

  //PID for rotators
  // One slot and one PID_IDX
  private static final int ROTATOR_TIMEOUT = 30; //ms
  private static final int ROTATOR_SLOT_IDX = 0;
  private static final int ROTATOR_PID_IDX = 0;
  private static final double ROTATOR_KF = 0.5;
  private static final double ROTATOR_KP = 0.02;
  private static final double ROTATOR_KI = 0;
  private static final double ROTATOR_KD = 0.01;

  //PID for main motors
  private static final int MAIN_MOTOR_TIMEOUT = 30; //ms
  private static final int MAIN_MOTOR_SLOT_IDX = 0;
  private static final int MAIN_MOTOR_PID_IDX = 0;
  private static final double MAIN_MOTOR_KF = 0.5;
  private static final double MAIN_MOTOR_KP = 0.02;
  private static final double MAIN_MOTOR_KI = 0;
  private static final double MAIN_MOTOR_KD = 0.01;

  //Kinematics
  SwerveDriveKinematics kinematics;
  Translation2d leftFront, leftBack, rightFront, rightBack;
  private static final double LEFT_FRONT_X = 0.25;
  private static final double LEFT_FRONT_Y = 0.25;
  private static final double LEFT_BACK_X = 0.25;
  private static final double LEFT_BACK_Y = 0.25;
  private static final double RIGHT_FRONT_X = 0.25;
  private static final double RIGHT_FRONT_Y = 0.25;
  private static final double RIGHT_BACK_X = 0.25;
  private static final double RIGHT_BACK_Y = 0.25;



  private void SwerveDriveSubsystemSetup() {
        // motor configuration block
    leftPrimary.configFactoryDefault();
    leftSecondary.configFactoryDefault();
    rightPrimary.configFactoryDefault();
    rightSecondary.configFactoryDefault();
    leftPrimaryRotator.configFactoryDefault();
    leftSecondaryRotator.configFactoryDefault();
    rightPrimaryRotator.configFactoryDefault();
    rightSecondaryRotator.configFactoryDefault();

    rightPrimary.setInverted(RIGHT_FRONT_MOTOR_INVERTED);
    rightSecondary.setInverted(RIGHT_BACK_MOTOR_INVERTED);
    rightPrimaryRotator.setInverted(RIGHT_FRONT_MOTOR_ROTATOR_INVERTED);
    rightSecondaryRotator.setInverted(RIGHT_BACK_MOTOR_ROTATOR_INVERTED);

    leftPrimary.setInverted(LEFT_FRONT_MOTOR_INVERTED);
    leftSecondary.setInverted(LEFT_BACK_MOTOR_INVERTED);
    leftPrimaryRotator.setInverted(LEFT_FRONT_MOTOR_ROTATOR_INVERTED);
    leftSecondaryRotator.setInverted(LEFT_BACK_MOTOR_ROTATOR_INVERTED);

    // Sets the direction that the talon will turn on the green LED when going 'forward'.
    leftPrimary.setSensorPhase(LEFT_FRONT_MOTOR_SENSOR_PHASE);
    rightPrimary.setSensorPhase(RIGHT_FRONT_MOTOR_SENSOR_PHASE);
    //Might need to add more for rotator motors. 

    // Current limits in amps
    // TODO: Find equivalent method names for FX stuff
    leftPrimary.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0));
    leftPrimary.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0));

    rightPrimary.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0));
    rightPrimary.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0));

    leftPrimaryRotator.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0));
    leftPrimaryRotator.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0));

    leftSecondaryRotator.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0));
    leftSecondaryRotator.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0));

    leftSecondary.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0));
    leftSecondary.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0));

    rightPrimaryRotator.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0));
    rightPrimaryRotator.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0));

    rightSecondary.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0));
    rightSecondary.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0));

    rightSecondaryRotator.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0));
    rightSecondaryRotator.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0));

    // Might need more for rotator motors

    /* TODO: Deal with motor controller faults once a physical robot is available for testing
    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();
    */

    leftPrimaryRotator.selectProfileSlot(ROTATOR_SLOT_IDX, ROTATOR_PID_IDX);
    leftPrimaryRotator.config_kF(ROTATOR_SLOT_IDX, ROTATOR_KF);
    leftPrimaryRotator.config_kP(ROTATOR_SLOT_IDX, ROTATOR_KP);
    leftPrimaryRotator.config_kI(ROTATOR_SLOT_IDX, ROTATOR_KI);
    leftPrimaryRotator.config_kD(ROTATOR_SLOT_IDX, ROTATOR_KD);

    rightPrimaryRotator.selectProfileSlot(ROTATOR_SLOT_IDX, ROTATOR_PID_IDX);
    rightPrimaryRotator.config_kF(ROTATOR_SLOT_IDX, ROTATOR_KF);
    rightPrimaryRotator.config_kP(ROTATOR_SLOT_IDX, ROTATOR_KP);
    rightPrimaryRotator.config_kI(ROTATOR_SLOT_IDX, ROTATOR_KI);
    rightPrimaryRotator.config_kD(ROTATOR_SLOT_IDX, ROTATOR_KD);

    leftSecondaryRotator.selectProfileSlot(ROTATOR_SLOT_IDX, ROTATOR_PID_IDX);
    leftSecondaryRotator.config_kF(ROTATOR_SLOT_IDX, ROTATOR_KF);
    leftSecondaryRotator.config_kP(ROTATOR_SLOT_IDX, ROTATOR_KP);
    leftSecondaryRotator.config_kI(ROTATOR_SLOT_IDX, ROTATOR_KI);
    leftSecondaryRotator.config_kD(ROTATOR_SLOT_IDX, ROTATOR_KD);

    rightSecondaryRotator.selectProfileSlot(ROTATOR_SLOT_IDX, ROTATOR_PID_IDX);
    rightSecondaryRotator.config_kF(ROTATOR_SLOT_IDX, ROTATOR_KF);
    rightSecondaryRotator.config_kP(ROTATOR_SLOT_IDX, ROTATOR_KP);
    rightSecondaryRotator.config_kI(ROTATOR_SLOT_IDX, ROTATOR_KI);
    rightSecondaryRotator.config_kD(ROTATOR_SLOT_IDX, ROTATOR_KD);

    //Kinematics
    //order is leftfront, leftback, rightfront, rightback
    leftFront = new Translation2d(LEFT_FRONT_X, LEFT_FRONT_Y);
    leftBack = new Translation2d(LEFT_BACK_X, LEFT_BACK_Y);
    rightFront = new Translation2d(RIGHT_FRONT_X, RIGHT_FRONT_Y);
    rightBack = new Translation2d(RIGHT_BACK_X, RIGHT_BACK_Y);
    kinematics = new SwerveDriveKinematics(leftFront, leftBack, rightFront, rightBack);
  }
  /**
   * Creates a new SwerveSubsystem.
   */
  public SwerveDriveSubsystem() 
  {
    SwerveDriveSubsystemSetup();
  }

  //Might need extra parameters for rotator motors
  //Make like differential drive subsystem constructor
  public SwerveDriveSubsystem(TalonFXConfiguration leftConfig, TalonFXConfiguration rightConfig)
  {
    leftPrimary.configAllSettings(leftConfig);
    leftSecondary.configAllSettings(leftConfig);
    rightPrimary.configAllSettings(rightConfig);
    rightSecondary.configAllSettings(rightConfig);

    SwerveDriveSubsystemSetup();
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


    leftPrimary.set(ControlMode.Position, modules[0].speedMetersPerSecond/METERS_PER_TICK);
    leftSecondary.set(ControlMode.Position, modules[1].speedMetersPerSecond/METERS_PER_TICK);
    rightPrimary.set(ControlMode.Position, modules[2].speedMetersPerSecond/METERS_PER_TICK);
    rightSecondary.set(ControlMode.Position, modules[3].speedMetersPerSecond/METERS_PER_TICK);

    leftPrimaryRotator.set(ControlMode.MotionMagic, modules[0].angle.getDegrees()*(4096/360));
    leftSecondaryRotator.set(ControlMode.MotionMagic, modules[1].angle.getDegrees()*(4096/360));
    rightPrimaryRotator.set(ControlMode.MotionMagic, modules[2].angle.getDegrees()*(4096/360));
    rightSecondaryRotator.set(ControlMode.MotionMagic, modules[3].angle.getDegrees()*(4096/360));
  } 

  public void setOutput(double omega, double XVelocity, double YVelocity)
  {
    setOutput(new ChassisSpeeds(XVelocity, YVelocity, omega));
  }

  /**
   * Enables brake.
   */
  public void enableBrakes() {
    leftPrimary.setNeutralMode(NeutralMode.Brake);
    rightPrimary.setNeutralMode(NeutralMode.Brake);
    leftSecondary.setNeutralMode(NeutralMode.Brake);
    rightSecondary.setNeutralMode(NeutralMode.Brake);
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
    leftSecondaryRotator.setNeutralMode(NeutralMode.Brake);
    rightSecondaryRotator.setNeutralMode(NeutralMode.Brake);
    leftSecondaryRotator.setNeutralMode(NeutralMode.Brake);
    rightSecondaryRotator.setNeutralMode(NeutralMode.Brake);
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
