package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @author Archan
 * @author Deepu
 * @author Abhi
 */
public class ShooterSubsystem extends SubsystemBase{

     // Physical parameters
  public static final double ENCODER_TICKS = 4096; // Ticks/rotation of CTREMagEncoder
  public static final double WHEEL_RADIUS_METERS = 0.1016;
  public static final double TICKS_PER_METER = ENCODER_TICKS/(Math.PI*WHEEL_RADIUS_METERS*WHEEL_RADIUS_METERS);

  // Port numbers to be added later
  //private static final int ROTATOR_PORT = 1;
  private static final int PROPELLOR_PORT = 2;

  //PID Slots
 // private static final int ROTATOR_SLOT_IDX = 0;
  //private static final int ROTATOR_PID_IDX = 0;
  private static final int PROPELLOR_PID_IDX = 0;
  private static final int PROPELLOR_SLOT_IDX = 0;

  private static final double PROPELLOR_KP = 1;
  private static final double PROPELLOR_KI = 0;
  private static final double PROPELLOR_KD = 0;
  private static final double PROPELLOR_KF = 0;

  // Physical limits
  private static final double MAX_SPEED = 10; // meters/sec
  private static final int PEAK_CURRENT_LIMIT = 60;
  private static final int CONTINUOUS_CURRENT_LIMIT = 50;

  private static final SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true,
  CONTINUOUS_CURRENT_LIMIT, 0, 0);
  

  // Phoenix Physics Sim Variables
  private static final double FULL_ACCEL_TIME = 0.75; // sec
  private static final double MAX_MOTOR_VEL = 4000; // ticks/(100ms)

  // Physical and Simulated Hardware
  // These talon objects are also simulated
  //private static final WPI_TalonSRX rotator = new WPI_TalonSRX(ROTATOR_PORT)
    private static final WPI_TalonSRX propellor = new WPI_TalonSRX(PROPELLOR_PORT);

    public ShooterSubsystem(TalonSRXConfiguration rotatorConfig, TalonSRXConfiguration propellorConfig)
    {  
     //rotator.configAllSettings(rotatorConfig);
      propellor.configAllSettings(propellorConfig);

      //rotator.selectProfileSlot(ROTATOR_SLOT_IDX, ROTATOR_PID_IDX);
      propellor.selectProfileSlot(PROPELLOR_SLOT_IDX, PROPELLOR_PID_IDX);

      propellor.config_kP(PROPELLOR_SLOT_IDX, PROPELLOR_KP);
      propellor.config_kI(PROPELLOR_SLOT_IDX, PROPELLOR_KI);
      propellor.config_kD(PROPELLOR_SLOT_IDX, PROPELLOR_KD);
      propellor.config_kF(PROPELLOR_SLOT_IDX, PROPELLOR_KF);

      propellor.configSupplyCurrentLimit(supplyCurrentLimit);


    }
    
    public void setFactoryMotorConfig()
    {
      //rotator.configFactoryDefault();
      propellor.configFactoryDefault();
    }


    public void setOutput(double desiredVelocity){
      propellor.set(ControlMode.Velocity, desiredVelocity*TICKS_PER_METER);
    }   
   

}
