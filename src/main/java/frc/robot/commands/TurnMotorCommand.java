package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.XboxMap;


public class TurnMotorCommand extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private WPI_TalonSRX motor = new WPI_TalonSRX(3);
    private double motorVelocity;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TurnMotorCommand(WPI_TalonSRX motor) {
      this.motor = motor;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.setSensorPhase(false);
        motor.configPeakCurrentLimit(60);
        motor.configContinuousCurrentLimit(50);
        motor.enableCurrentLimit(true);

        motorVelocity = 0;
        SmartDashboard.putNumber("output", 0);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
        motorVelocity = XboxMap.left();
        motor.set(ControlMode.PercentOutput, motorVelocity);
        SmartDashboard.putNumber("output", motor.getSelectedSensorPosition(1));
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println(motor.getSelectedSensorPosition(1));
        motor.set(ControlMode.PercentOutput, 0);
        System.out.println("Manual motor spinning stopped.");
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
