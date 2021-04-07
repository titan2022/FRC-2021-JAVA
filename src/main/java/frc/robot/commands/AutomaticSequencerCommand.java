package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SequencerSubsystem;

public class AutomaticSequencerCommand extends CommandBase{
    private static SequencerSubsystem SequencerSubsystem;

    private double motorVelocityTicks;

    public AutomaticSequencerCommand(SequencerSubsystem sequencerSubsystem){
        addRequirements(sequencerSubsystem);
        this.SequencerSubsystem = SequencerSubsystem;
    }

    


    public void initialize()
    {
        System.out.println("Sequencer command started.");
    }

    public void setVelocityMotors()
    {
        
    }

    public void getNumBalls()
    {

    }

    public void execute(){
        
    }
    
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
