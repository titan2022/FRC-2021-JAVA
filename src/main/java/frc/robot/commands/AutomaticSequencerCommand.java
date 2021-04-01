package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SequencerSubsystem;

public class AutomaticSequencerCommand extends CommandBase{
    private static SequencerSubsystem SequencerSubsystem;
    public AutomaticSequencerCommand(SequencerSubsystem sequencerSubsystem){
        addRequirements(sequencerSubsystem);
        SequencerSubsystem = sequencerSubsystem;
    }

    public void sequencerLimit(int numBalls){

    }

    


    public void initialize()
    {
        System.out.println("Automatic Sequencer Command Started")

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
