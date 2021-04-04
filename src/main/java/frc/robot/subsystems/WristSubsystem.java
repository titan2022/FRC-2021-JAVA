package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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
    private final int PCM_PORT = 62;

    private final DoubleSolenoid doubleSol = new DoubleSolenoid(PCM_PORT, 1,2); // TODO: figure out the PCM port and channels 

    /**
     * @param motorConfig - specific configurations for wrist 
     */
    public WristSubsystem() {
        doubleSol.set(DoubleSolenoid.Value.kOff);       
    }
    
    /**
     * Extends scissor intake
     */
    public void extend() {
        doubleSol.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Retracts the scissor intake
     */
    public void retract() {
        doubleSol.set(DoubleSolenoid.Value.kReverse);
    }
    /**
     * Releases the pressure from the pistons
     */
    public void stop() {
        doubleSol.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public void periodic()
    {

    }
}
