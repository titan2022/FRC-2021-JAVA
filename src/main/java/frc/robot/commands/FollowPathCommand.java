package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motion.generation.rmpflow.GoalAttractor;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPathCommand extends CommandBase {
    private final PathCommand planner;
    private final MotionGenerationCommand motion;
    public final DriveSubsystem drive;
    private GoalAttractor attractor;
    private double stall;
    private boolean done = false;

    public FollowPathCommand(PathCommand planner, MotionGenerationCommand motion, DriveSubsystem drive, double stall) {
        this.planner = planner;
        this.motion = motion;
        this.drive = drive;
        this.stall = stall;
        attractor = new GoalAttractor("Path goal", motion.motion.getRoot(), toMatrix(motion.filter.getFilteredPose()), 10, 1, 10, 1, 2, 2, 0.005);
        motion.motion.addGoalAttractor(attractor);
    }

    private SimpleMatrix toMatrix(Pose2d x) {
        return new SimpleMatrix(new double[][]{{x.getX()}, {x.getY()}, {x.getRotation().getRadians()}});
    }
}
