package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.CircularArc;
import frc.robot.mapping.CompoundPath;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Path;
import frc.robot.mapping.Point;
import frc.robot.motion.generation.rmpflow.GoalAttractor;
import frc.robot.path.dstar.DStarLite;
import frc.robot.subsystems.MotionGenerationSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class PathCommand extends CommandBase {
    public final NavigationSubsystem nav;
    public final DStarLite planner;
    public final MotionGenerationSubsystem rmp;
    public final GoalAttractor goal;

    public PathCommand(Point target, ObstacleMap map, NavigationSubsystem nav, MotionGenerationSubsystem rmp, double radius) {
        this(target, map, nav, rmp, radius, null);
    }
    public PathCommand(Point target, ObstacleMap map, NavigationSubsystem nav, MotionGenerationSubsystem rmp, double radius, DStarLite planner) {
        this.nav = nav;
        this.planner = planner == null ? new DStarLite(map, target, radius) : planner;
        this.rmp = rmp;
        SimpleMatrix pos = new SimpleMatrix(new double[][]{{target.getX()}, {target.getY()}});
        this.goal = new GoalAttractor("Path step", rmp.getRoot(), pos, 10, 1, 10, 1, 2, 2, 0.005);
    }

    private Point firstEndpoint(Path path){
        if(path instanceof CompoundPath){
            for(Path segment : ((CompoundPath) path).getSegments())
                if(segment.getLength() > 0.1)
                    return firstEndpoint(segment);
        }
        else if(path instanceof CircularArc){
            double radPerLen = Math.abs(((CircularArc) path).getAngularVelocity(0).getRadians());
            if(path.getLength() * radPerLen > Math.PI / 6)
                return path.getPos(Math.PI / (6*radPerLen));  // 30 degrees
        }
        return path.getEnd();
    }

    public void execute() {
        Point pos = new Point(nav.getUnfilteredX(), nav.getUnfilteredY());
        planner.setStart(pos);
        Point target = firstEndpoint(planner.getPath());
        goal.updateGoal(new SimpleMatrix(new double[][]{{target.getX()}, {target.getY()}}));
    }
}
