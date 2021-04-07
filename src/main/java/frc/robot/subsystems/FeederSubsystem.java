package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import jdk.internal.net.http.frame.FramesDecoder;

public class FeederSubsystem extends SubsystemBase{
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
    private static final int CONTINUOUS_CURRENT_LIMIT = 50;

    private static final SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true,
    CONTINUOUS_CURRENT_LIMIT, 0, 0);

    // Physical and Simulated Hardware
  // These talon objects are also simulated
  //private static final WPI_TalonSRX rotator = new WPI_TalonSRX(ROTATOR_PORT)
  private static final WPI_TalonSRX feeder = new WPI_TalonSRX(SEQUENCER_PORT);
  
  public FeederSubsystem(TalonSRXConfiguration sequencerConfig)
  {
    feeder.configAllSettings(sequencerConfig);

    feeder.selectProfileSlot(SEQUENCER_SLOT_IDX, SEQUENCER_PID_IDX);

    feeder.config_kP(SEQUENCER_SLOT_IDX, SEQUENCER_KP);
    feeder.config_kI(SEQUENCER_SLOT_IDX, SEQUENCER_KI);
    feeder.config_kD(SEQUENCER_SLOT_IDX, SEQUENCER_KD);
    feeder.config_kF(SEQUENCER_SLOT_IDX, SEQUENCER_KF);

    feeder.configSupplyCurrentLimit(supplyCurrentLimit);

  }

  public void setFactoryMotorConfig()
  {
    FramesDecoder.configFactoryDefault();
  }

  public void setOutput(double speed)
  {
   feeder.set(ControlMode.PercentOutput,speed*TICKS_PER_METER);   
  }
}

