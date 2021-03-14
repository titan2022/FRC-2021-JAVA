package frc.robot.config;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.VisionProcessingCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.RedShirtPipeline;

/** Add your docs here. */
public class GalacticSearchContainer implements RobotContainer {
  private VisionSubsystem visionSub;

  private final ParallelCommandGroup autoGroup, teleopGroup;

  public GalacticSearchContainer() {
    visionSub = new VisionSubsystem();

    autoGroup = new ParallelCommandGroup(); // These don't actually run in parallel.
    teleopGroup = new ParallelCommandGroup();

    VisionProcessingCommand processingCommand = new VisionProcessingCommand(visionSub, new RedShirtPipeline());

    teleopGroup.addCommands(processingCommand);
  }

  public Command getAutonomousCommand() {

    return autoGroup;

  };

  public Command getTeleopCommand() {

    return teleopGroup;

  };

}
