package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * 
 */
public class VHopperSubsystem extends SubsystemBase {
  public static final int MOTOR_PORT = 5;

  private static final SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 20, 0, 0);
  private static final StatorCurrentLimitConfiguration statorCurrentLimit = new StatorCurrentLimitConfiguration(true, 30, 0, 0);
  
  private static final boolean MOTOR_INVERTED = false;

  public static final WPI_TalonFX motor = new WPI_TalonFX(MOTOR_PORT);

  public VHopperSubsystem(TalonFXConfiguration userConfig) {
    motor.configFactoryDefault();

    if(userConfig != null)
      motor.configAllSettings(userConfig);
    
    motor.setInverted(MOTOR_INVERTED);
    motor.configSupplyCurrentLimit(supplyCurrentLimit);
    motor.configStatorCurrentLimit(statorCurrentLimit);
    //motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0); // It is integrated by default
  }

  public VHopperSubsystem(){
    this(null);
  }

  /**
   * Sets each of the motor speeds
   *
   * @param speed  The target speed as a percent of the max speed.
   */
  public void setOutputs(double value) {
    motor.set(ControlMode.PercentOutput, value);
  }

  /**
   * Stops the motors.
   */
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
