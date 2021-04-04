package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;

/**
 * 
 */
public class ManualWristCommand extends CommandBase {
    private IntakeSubsystem intakeSub; 
    private boolean up = false;
  
    /**
     *
     * @param subsystem The subsystem used by this command.
     */
    public ManualWristCommand(IntakeSubsystem intakeSub) {
      addRequirements(intakeSub);
      this.intakeSub = intakeSub;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("Wrist Command Started");
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {            
        if (XboxMap.toggleWrist()) {up = !up;}

        if(up) {
          intakeSub.extend();
        }
        else {
          intakeSub.retract();
        }
        intakeSub.setMotorSpeed(XboxMap.scissorSpeed());
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSub.stop();
        System.out.println("Wrist Command Stopped");
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}