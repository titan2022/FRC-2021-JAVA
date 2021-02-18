package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
/**
 * @author Irene
 */
public class WristSubsystem extends SubsystemBase {
    
    public static final int PRIMARY_WRIST_PORT = 3;
    public static final int SECONDARY_WRIST_PORT = 1;

    public static final boolean PRIMARY_WRIST_PORT_INVERTED = false;
    public static final boolean SECONARDY_WRIST_PORT_INVERTED = false;

    private static final boolean PRIMARY_WRIST_SENSOR_PHASE = false;

    private static final int ENCODER_PORT = 0;
    
    private static final double ANGLE_LOWER_LIMIT = 10; // TODO: Get from DI Team
    private static final double ANGLE_UPPER_LIMIT = 60; // TODO: Get from DI Team
    private static final int PEAK_CURRENT_LIMIT = 60;
    private static final int CONTINUOUS_CURRENT_LIMIT = 50;
    public static final double ENCODER_TICKS = 4096; 
    public static final double ANGLE_TO_TICK =  1 / (360 * ENCODER_TICKS);

    // PID Constants
    private static final double WRIST_KP = 2;
    private static final double WRIST_KI = .5;
    private static final double WRIST_KD = 1;
    private static final double WRIST_KF = 0;

    private static final WPI_TalonSRX wrist = new WPI_TalonSRX(PRIMARY_WRIST_PORT);;
    private static final WPI_TalonSRX followingWrist = new WPI_TalonSRX(SECONDARY_WRIST_PORT); //like a left wrist

    
    public void setMotorFactoryConfig()
    {
        wrist.configFactoryDefault();
        followingWrist.configFactoryDefault();
    }

    /**
     * @param motorConfig - specific configurations for wrist 
     */
    public WristSubsystem(TalonSRXConfiguration motorConfig) {
        setMotorFactoryConfig();
        if(motorConfig != null)
        {
            wrist.configAllSettings(motorConfig);
            followingWrist.configAllSettings(motorConfig);
        }

        // Motor configs
        wrist.config_kP(0, WRIST_KP);
        wrist.config_kI(0, WRIST_KI);
        wrist.config_kD(0, WRIST_KD);
        wrist.config_kF(0, WRIST_KF);
        followingWrist.config_kP(0, WRIST_KP);
        followingWrist.config_kI(0, WRIST_KI);
        followingWrist.config_kD(0, WRIST_KD);
        followingWrist.config_kF(0, WRIST_KF);
        wrist.setSensorPhase(PRIMARY_WRIST_SENSOR_PHASE);
        followingWrist.setSensorPhase(PRIMARY_WRIST_SENSOR_PHASE);

        wrist.setInverted(PRIMARY_WRIST_PORT_INVERTED);
        followingWrist.setInverted(PRIMARY_WRIST_PORT_INVERTED);

        wrist.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);
        wrist.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);
        wrist.configPeakCurrentDuration(10); 
        wrist.enableCurrentLimit(true);

        followingWrist.follow(wrist);
        

        //Encoder configs
        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        
    }

    public WristSubsystem()
    {
        this(null);
    }

    /**
     * 
     * @param angle desired angle to turn (in degrees)
     */
    public void setWristPosition(double angle)
    {
        wrist.set(ControlMode.MotionProfile, angle * ANGLE_TO_TICK);
    }
    /**
     * 
     * @param Velocity desired velocity to turn at
     */
    public void setWristVel(double Velocity)
    {
        wrist.set(ControlMode.Velocity, Velocity * ANGLE_TO_TICK);
    }
    /**
     * 
     * @param percentOut desired percent output (between -1 and 1)
     */
    public void setPercentOutput(double percentOut) {
        
        wrist.set(ControlMode.PercentOutput, percentOut);
    }

    /**
     * 
     * @return current angle of wrist
     */
    public double getWristAngle() {
        return wrist.getSelectedSensorPosition();
    }
    
    public void checkWristLimits() {
        if (getWristAngle() < ANGLE_LOWER_LIMIT) {
            setWristPosition(ANGLE_LOWER_LIMIT);
        }
                
        if (getWristAngle() > ANGLE_UPPER_LIMIT) {
            setWristPosition(ANGLE_UPPER_LIMIT);
        }
    }
    public void stop() {
        wrist.set(ControlMode.PercentOutput, 0);
    }
    @Override
    public void periodic()
    {
        checkWristLimits();
    }
}
