package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Point;
import frc.robot.motion.generation.rmpflow.GoalAttractor;
import frc.robot.path.dstar.DStarLite;
import frc.robot.subsystems.MotionGenerationSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class PathCommand extends CommandBase {
    final NavigationSubsystem nav;
    final DStarLite planner;
    final MotionGenerationSubsystem rmp;
    final GoalAttractor goal;

    public PathCommand(Point target, ObstacleMap map, NavigationSubsystem nav, MotionGenerationSubsystem rmp, double radius) {
        this.nav = nav;
        this.planner = new DStarLite(map, target, radius);
        this.rmp = rmp;
        SimpleMatrix pos = new SimpleMatrix(new double[][]{{target.getX()}, {target.getY()}});
        this.goal = new GoalAttractor("Path step", rmp.getRoot(), pos, 10, 1, 10, 1, 2, 2, 0.005);
    }
}
