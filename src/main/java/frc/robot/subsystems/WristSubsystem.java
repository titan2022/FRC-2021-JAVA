package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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

    // Motion Magic Constants
    // PID Constants
    private static final double WRIST_KP = 2;
    private static final double WRIST_KI = .5;

    private static final WPI_TalonSRX wrist = new WPI_TalonSRX(PRIMARY_WRIST_PORT);;
    private static final WPI_TalonSRX followingWrist = new WPI_TalonSRX(SECONDARY_WRIST_PORT); //like a left wrist

    private double wristAngle, wristSpeed;


    /**
     * 
     */
    public WristSubsystem(TalonSRXConfiguration motorConfig) {
        setMotorFactoryConfig();

        if(motorConfig != null)
        {
            wrist.configAllSettings(motorConfig);
            followingWrist.configAllSettings(motorConfig);
        }

        // Motor configs
        wrist.setInverted(PRIMARY_WRIST_PORT_INVERTED);
        followingWrist.setInverted(PRIMARY_WRIST_PORT_INVERTED);

        wrist.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);
        wrist.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);
        //wrist.configPeakCurrentDuration(10); 
        wrist.enableCurrentLimit(true);

        followingWrist.follow(wrist);

        //Encoder configs
    }

    public WristSubsystem()
    {
        this(null);
    }
    public void setMotorFactoryConfig()
    {
        wrist.configFactoryDefault();
        followingWrist.configFactoryDefault();
    }
    
    public void setWristPosition(double angles)
    {
        
    }






    /**
     * Returns the linear speed of the write // TODO: Check if wrist speed is angular are linear
     * @return The wrist speed in meter / s
     */
    public double getWristSpeed() {
        return wristSpeed;
    }
    
    /**
     * 
     * @param speed
     */
    public void setWristSpeed(double speed) {
        wrist.set(ControlMode.PercentOutput, speed); //makes wrist speed desired speed
    }

    //angle
    public double getWristAngle() {
        return wristAngle;
    }
    public void rotateUp(double degrees) {
        wrist.set(ControlMode.MotionMagic, degrees); //rotates the amount of degrees specifed
    }

    public void rotateDown(double degrees) {
        wrist.set(ControlMode.MotionMagic, degrees); //rotates the amount of degrees specifed
    }
    public void checkAngle() {
        if(wristAngle > angleLimit) {
            wrist.set(ControlMode.MotionMagic, angleLimit-1);
        }
    }

    public void stop() {
        wrist.set(ControlMode.PercentOutput, 0); //stops the wrist from moving

    }
    
    @Override
    public void periodic()
    {

    }
}