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
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class VHopperSubsystem extends SubsystemBase {

  public static final int MOTOR_PORT = 5; //Motor 1: bottom of conveyor

  public static final double WHEEL_RADIUS = 0.01;  // TODO: get correct value in meters
  public static final double TICKS_PER_METER = 2048 / (WHEEL_RADIUS * 2 * Math.PI);

  public static final int MAX_OUTPUT = 0; //TODO determine MAX_OUTPUT or remove
  private static final int CONTINUOUS_CURRENT_LIMIT = 0;
  private static final int PEAK_CURRENT_LIMIT = 0;
  private static final SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, 0, 0);
  private static final StatorCurrentLimitConfiguration statorCurrentLimit = new StatorCurrentLimitConfiguration(true, PEAK_CURRENT_LIMIT, 0, 0);
  
  private static final boolean MOTOR_INVERTED = false;
  private static final boolean MOTOR_SENSOR_PHASE = false;

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


  public static final WPI_TalonSRX motor =new WPI_TalonSRX(MOTOR_PORT);

  
  public VHopperSubsystem(TalonSRXConfiguration userConfig) {
    motor.configFactoryDefault();
    if(userConfig != null)
      motor.configAllSettings(userConfig);
    motor.setInverted(MOTOR_INVERTED);
    motor.setSensorPhase(MOTOR_SENSOR_PHASE);
    motor.configSupplyCurrentLimit(supplyCurrentLimit);
    //motor.configStatorCurrentLimit(statorCurrentLimit);
    motor.selectProfileSlot(SLOT_IDX, PID_IDX);
    //motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
  }
  public VHopperSubsystem(){
    this(null);
  }


  /**
   * Sets each of the motor speeds
   *
   * @param speed  The target speed as a percent of the max speed.
   */
  public void setOutputs(double speed) {
    //TODO: determine MAX_OUTPUT & how the power distribution will work
    //TODO: add any required checks
    motor.set(ControlMode.PercentOutput, speed);
    
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
    motor.set(ControlMode.PercentOutput, 0);

  }


  /**
   * Enables brake.
   */
  public void enableBrakes() {
    motor.setNeutralMode(NeutralMode.Brake);

  }

  /**
   * Disables brake.
   */
  public void disableBrakes() {
    motor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
