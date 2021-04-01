package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SequencerSubsystem;

public class AutomaticSequencerCommand extends CommandBase{
    private static SequencerSubsystem SequencerSubsystem;
    public AutomaticSequencerCommand(SequencerSubsystem sequencerSubsystem){
        addRequirements(sequencerSubsystem);
        SequencerSubsystem = sequencerSubsystem;
    }

    


    public void initialize()
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
