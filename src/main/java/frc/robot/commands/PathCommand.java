package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.Point;
import frc.robot.path.dstar.DStarLite;

public class PathCommand extends CommandBase {
    public final DStarLite planner;
    public final DifferentialDriveFilterCommand filter;

    public PathCommand(DStarLite planner, DifferentialDriveFilterCommand filter) {
        this.planner  = planner;
        this.filter = filter;
    }

    public void execute() {
        planner.setStart(new Point(filter.getFilteredPose().getTranslation()));
    }
}
