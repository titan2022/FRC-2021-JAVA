package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;

/**
 * @author Irene
 */
public class ManualWristCommand extends CommandBase {
    private static WristSubsystem wristSubsystem; 
    private boolean up = false;
  
    /**
     *
     * @param subsystem The subsystem used by this command.
     */
    public ManualWristCommand(WristSubsystem subsystem) {
      addRequirements(subsystem);
      wristSubsystem = subsystem;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("Wrist Command Started");
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //wristSubsystem.setWristPosition(XboxMap.spinWrist());
            
        if (XboxMap.toggleWrist()) {up = !up;}
        if(up) {
          wristSubsystem.goUp();
        }
        else {
          wristSubsystem.goDown();
        }
        
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.stop();
        System.out.println("Wrist Command Stopped");
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}