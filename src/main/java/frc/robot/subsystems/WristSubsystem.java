package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * @author Irene
 */
public class WristSubsystem extends SubsystemBase {
    private final int PCM_PORT = 62;
    private final int MOTOR_PORT = 0;

    private final DoubleSolenoid doubleSol = new DoubleSolenoid(PCM_PORT, 1,2); // TODO: figure out the PCM port and channels
    private final WPI_TalonFX motor = new WPI_TalonFX(MOTOR_PORT);

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
     * Sets the intake motor speed.
     * 
     * @param value  The target speed, as a percent of max speed.
     */
    public void setMotorSpeed(double value) {
        motor.set(ControlMode.PercentOutput, value);
    }

    /**
     * Releases the pressure from the pistons
     */
    public void stop() {
        doubleSol.set(DoubleSolenoid.Value.kOff);
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic()
    {

    }
}
