package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.XboxMap;

/**
 * @author Irene
 */
public class WristCommand extends CommandBase {
    private static WristSubsystem wristSubsystem; 
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public WristCommand(WristSubsystem subsystem) {
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
        if (XboxMap.goWristUp()) {
            wristSubsystem.setWristPosition(90);
            if (XboxMap.stopWristUp()) {
                wristSubsystem.stop();
            }
        }
        if (XboxMap.goWristDown()) {
            wristSubsystem.setWristPosition(0);
            if (XboxMap.stopWristDown()) {
                wristSubsystem.stop();
            }
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