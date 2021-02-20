package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.Point;
import frc.robot.path.dstar.DStarLite;

public class PathCommand extends CommandBase {
    public final DStarLite planner;
    public final DifferentialDriveOdometryCommand odometry;

    public PathCommand(DStarLite planner, DifferentialDriveOdometryCommand odometry) {
        this.planner  = planner;
        this.odometry = odometry;
    }

    public void execute() {
        Point pos = new Point(odometry.getX(), odometry.getY());
        planner.setStart(pos);
    }
}
