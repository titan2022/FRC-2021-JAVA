package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.XboxMap;
import frc.robot.subsystems.VHopperSubsystem;

public class ManualVHopperCommand extends CommandBase {
    public final VHopperSubsystem vhopper;
    public final double fullSpeed;

    public ManualVHopperCommand(VHopperSubsystem vhopper, double fullSpeed) {
        this.vhopper = vhopper;
        this.fullSpeed = fullSpeed;
    }
    public ManualVHopperCommand(VHopperSubsystem vhopper) {
        this(vhopper, 1.0);  // TODO: Replace with appropriate default value.
    }

    @Override
    public void execute() {
        double targetSpeed = XboxMap.hopperPct() * fullSpeed;
        vhopper.setOutputs(targetSpeed, targetSpeed);
    }
}
