package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SequencerSubsystem extends SubsystemBase{
      // Physical parameters
    public static final double ENCODER_TICKS = 4096; // Ticks/rotation of CTREMagEncoder
    public static final double WHEEL_RADIUS_METERS = 0.1016;
    public static final double TICKS_PER_METER = ENCODER_TICKS/(Math.PI*WHEEL_RADIUS_METERS*WHEEL_RADIUS_METERS);

    //port numbers
    private static final int SEQUENCER_PORT = 1;

     //PID Slots
     private static final int SEQUENCER_PID_IDX = 0;
     private static final int SEQUENCER_SLOT_IDX = 0;

     private static final double SEQUENCER_KP = 1;
     private static final double SEQUENCER_KI = 0;
     private static final double SEQUENCER_KD = 0;
     private static final double SEQUENCER_KF = 0;

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
  private static final WPI_TalonSRX sequencer = new WPI_TalonSRX(SEQUENCER_PORT);
  
  public SequencerSubsystem(TalonSRXConfiguration sequencerConfig)
  {
    sequencer.configAllSettings(sequencerConfig);

    sequencer.selectProfileSlot(SEQUENCER_SLOT_IDX, SEQUENCER_PID_IDX);

    sequencer.config_kP(SEQUENCER_SLOT_IDX, SEQUENCER_KP);
    sequencer.config_kI(SEQUENCER_SLOT_IDX, SEQUENCER_KI);
    sequencer.config_kD(SEQUENCER_SLOT_IDX, SEQUENCER_KD);
    sequencer.config_kF(SEQUENCER_SLOT_IDX, SEQUENCER_KF);

    sequencer.configSupplyCurrentLimit(supplyCurrentLimit);

  }

  public void setFactoryMotorConfig()
  {
    sequencer.configFactoryDefault();
  }

  public void setOutput(double velocity)
  {
   sequencer.set(ControlMode.Velocity,velocity*TICKS_PER_METER);   
  }

}

