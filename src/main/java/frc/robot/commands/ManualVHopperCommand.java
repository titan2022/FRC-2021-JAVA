package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;
import frc.robot.subsystems.VHopperSubsystem;

public class ManualVHopperCommand extends CommandBase {
    public final VHopperSubsystem vhopper;

    public ManualVHopperCommand(VHopperSubsystem vhopper) {
        this.vhopper = vhopper;
    }

    @Override
    public void execute() {
        vhopper.setOutputs(XboxMap.hopperPct());
    }

    @Override
    public void end(boolean interrupted) {
        vhopper.stop();
    }
}
