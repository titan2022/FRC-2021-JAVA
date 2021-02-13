package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class WristSubsystem extends SubsystemBase {
    
    public static final int WRIST_PORT = 1;
    public static final int WRIST_PORT2 = 2;
    private TalonSRX wrist;
    private TalonSRX followingWrist;

    private double angleLimit;
    public int currentLimit = 10; 

    public double wristAngle, wristSpeed;

    public WristSubsystem() {
        wrist = new TalonSRX(WRIST_PORT);
        followingWrist = new TalonSRX(WRIST_PORT2);
        setupWrist();

    }
    public void setupWrist() {
        followingWrist.setInverted(true);
        followingWrist.follow(wrist);
        
        wrist.configContinuousCurrentLimit(currentLimit); 
        wrist.configPeakCurrentLimit(currentLimit);
        wrist.configPeakCurrentDuration(10); 
        wrist.enableCurrentLimit(true);
    }

    public double getWristSpeed() {
        return wristSpeed;
    }
    public double getMaxSpeed() {
        return currentLimit;
    }
    public void setWristSpeed(double speed) {
        wrist.set(ControlMode.PercentOutput, speed);
    }

    public double getWristAngle() {
        return wristAngle;
    }
    public void rotateUp(double degrees) {
        wrist.set(ControlMode.MotionMagic, degrees);
    }

    public void rotateDown(double degrees) {
        wrist.set(ControlMode.MotionMagic, degrees);
    }
    public void checkAngle() {
        if(wristAngle > angleLimit) {
            wrist.set(ControlMode.MotionMagic, angleLimit-1);
        }
    }
    public void stop() {
        wrist.set(ControlMode.PercentOutput, 0);

    }
    
}