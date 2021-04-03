/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import frc.robot.subsystems.sim.PhysicsSim;
//import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;



public class VHopperSubsystem extends SubsystemBase {

  public static final int MOTOR_B_PORT = 5; //Motor 1: bottom of conveyor
  public static final int MOTOR_TL_PORT = 6; //Motor 2: top left of conveyor
  public static final int MOTOR_TR_PORT = 7; //Motor 3: top right of conveyor

  public static final int MAX_OUTPUT = 0; //TODO determine MAX_OUTPUT
  private static final int CONTINUOUS_CURRENT_LIMIT = 0;
  private static final int PEAK_CURRENT_LIMIT = 0;
  private static final SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0);
  private static final StatorCurrentLimitConfiguration statorCurrentLimit = new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0);
  
  public static final boolean MOTOR_B_INVERTED = false;
  public static final boolean MOTOR_TL_INVERTED = false;
  public static final boolean MOTOR_TR_INVERTED = false;
  
  public static final boolean MOTOR_B_SENSOR_PHASE = false;
  public static final boolean MOTOR_TL_SENSOR_PHASE = false;
  public static final boolean MOTOR_TR_SENSOR_PHASE = false;

  private static final int SLOT_IDX = 0;
  private static final int PID_IDX = 0;

  public static final double BELT_LENGTH = 0.0;
  public static final double INCLINE_ANGLE = 0.0;
  public static final double TAIL_PULLEY_DIAMETER = 0.0;
  public static final double DRIVE_PULLEY_DIAMETER = 0.0;

  public static final double BELT_HEIGHT = 0.0; //Belt height
  public static final double BELT_LOAD = 0.0; // Load due to the Belt (balls) in Kg/m
  public static final double BALL_LOAD = 0.0; // Load due to the idlers (balls) in Kg/m
  public static final double DRIVE_EFFICENCY = 0.0;
  public static final double BELT_VELOCITY = 0.0; 


  public static final WPI_TalonFX MotorB =new WPI_TalonFX(MOTOR_B_PORT)
  , MotorTL = new WPI_TalonFX(MOTOR_TL_PORT)
  , MotorTR = new WPI_TalonFX(MOTOR_TR_PORT);

  
  public VHopperSubsystem(TalonFXConfiguration userConfig) {
    MotorB.configFactoryDefault();
    MotorTL.configFactoryDefault();
    MotorTR.configFactoryDefault();
    if(userConfig != null){
      MotorB.configAllSettings(userConfig);
      MotorTL.configAllSettings(userConfig);
      MotorTR.configAllSettings(userConfig);
    }
    MotorB.setInverted(MOTOR_B_INVERTED);
    MotorTL.setInverted(MOTOR_TL_INVERTED);
    MotorTR.setInverted(MOTOR_TR_INVERTED);
    MotorB.setSensorPhase(MOTOR_B_INVERTED);
    MotorTL.setSensorPhase(MOTOR_TL_INVERTED);
    MotorTR.setSensorPhase(MOTOR_TR_INVERTED);
    MotorB.configSupplyCurrentLimit(supplyCurrentLimit);
    MotorTL.configSupplyCurrentLimit(supplyCurrentLimit);
    MotorTR.configSupplyCurrentLimit(supplyCurrentLimit);
    MotorB.configStatorCurrentLimit(statorCurrentLimit);
    MotorTL.configStatorCurrentLimit(statorCurrentLimit);
    MotorTR.configStatorCurrentLimit(statorCurrentLimit);
    MotorB.selectProfileSlot(SLOT_IDX, PID_IDX);
    MotorTL.selectProfileSlot(SLOT_IDX, PID_IDX);
    MotorTR.selectProfileSlot(SLOT_IDX, PID_IDX);
    MotorB.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    MotorTL.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    MotorTR.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    MotorTL.follow(MotorTR);
  }
  public VHopperSubsystem(){
    this(null);
  }


    /**
   * Sets each of the motor outputs
   * 
   *
   * @param bottomOutput  bottom motor output value 
   * @param topOutput  top motors' output value
   */
  public void setMotorOutputs(double bottomOutput, double topOutput) {
    //TODO: determine MAX_OUTPUT & how the power distribution will work
    
    if (bottomOutput < MAX_OUTPUT) MotorB.set(bottomOutput);
    if (topOutput < MAX_OUTPUT) MotorTR.set(topOutput);
    
  }

  /**
   * Following methods are modulated for now to make heuristics easier. Their
   * conglomeration will eventually just be getMinimumMotorPower()
   */

  /**
   * Returns belt speed in FPM based on given RPM of tail pulley motors
   */
  public double getBeltSpeedAtRPM(double tailPulleyRPM) {
    return DRIVE_PULLEY_DIAMETER * Math.PI * tailPulleyRPM; 
  }

  //returns  belt tension from the aggregate of factors(Newtons)
  public double getBeltTension () {
    return 1.37 * BELT_LENGTH *  9.81 + ((2 * BELT_LOAD + BALL_LOAD) * Math.cos(INCLINE_ANGLE)) + (BELT_HEIGHT * 9.81 * BALL_LOAD); //TODO Access Physics constants: G (replace with 9.81) and coefficent of friction
  }

  //returns the power needed for the drive pulley
  public double getPowerAtDrive (double tailPulleyRPM) {
    return (getBeltTension() * getBeltSpeedAtRPM(tailPulleyRPM) / 1000 );
  }

   /**
   * Returns the minimum motor power needed for Drive pulley (hp)
   */
  public double getMinimumMotorPower (double tailPulleyRPM) {
    return (getPowerAtDrive(tailPulleyRPM) / DRIVE_EFFICENCY); 
    
  }
 
  

    /**
   * Stops the motors.
   */
  public void stop() {
    MotorB.set(ControlMode.PercentOutput, 0);
    MotorTR.set(ControlMode.PercentOutput, 0);

  }


  /**
   * Enables brake.
   */
  public void enableBrakes() {
    MotorB.setNeutralMode(NeutralMode.Brake);
    MotorTR.setNeutralMode(NeutralMode.Brake);

  }

  /**
   * Disables brake.
   */
  public void disableBrakes() {
    MotorB.setNeutralMode(NeutralMode.Coast);
    MotorTR.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
