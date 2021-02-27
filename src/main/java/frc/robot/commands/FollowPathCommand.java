package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.CircularArc;
import frc.robot.mapping.CompoundPath;
import frc.robot.mapping.Path;
import frc.robot.mapping.Point;
import frc.robot.motion.generation.rmpflow.GoalAttractor;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPathCommand extends CommandBase {
    private final PathCommand planner;
    private final MotionGenerationCommand motion;
    public final DriveSubsystem drive;
    public final double tolerance;
    private GoalAttractor attractor;

    public FollowPathCommand(PathCommand planner, MotionGenerationCommand motion, DriveSubsystem drive, double tolerance) {
        this.planner = planner;
        this.motion = motion;
        this.drive = drive;
        this.tolerance = tolerance;
        attractor = new GoalAttractor("Path goal", motion.root, toMatrix(motion.filter.getFilteredPose()), 10, 1, 10, 1, 2, 2, 0.005);
    }

    private SimpleMatrix toMatrix(Pose2d x) {
        return new SimpleMatrix(new double[][]{{x.getX()}, {x.getY()}, {x.getRotation().getRadians()}});
    }

    private SimpleMatrix firstEndpoint(Path path){
        if(path instanceof CompoundPath){
            for(Path segment : ((CompoundPath) path).getSegments())
                if(segment.getLength() > 0.1)
                    return firstEndpoint(segment);
        }
        else if(path instanceof CircularArc){
            double radPerLen = Math.abs(((CircularArc) path).getAngularVelocity(0).getRadians());
            double d = Math.PI / (6 * radPerLen);
            if(path.getLength() > d){
                Point x = path.getPos(d);
                return new SimpleMatrix(new double[][]{{x.getX()}, {x.getY()}, {path.getRotation(d).getRadians()}});
            }
        }
        Point x = path.getEnd();
        return new SimpleMatrix(new double[][]{{x.getX()}, {x.getY()}, {path.getRotation(path.getLength()).getRadians()}});
    }

    @Override
    public void execute() {
        attractor.updateGoal(firstEndpoint(planner.planner.getPath()));
        Translation2d vel = motion.getVelocity();
        double theta_dot = motion.getRotationalVelocity();
        drive.setVelocities(new ChassisSpeeds(vel.getX(), vel.getY(), theta_dot));
    }

    @Override
    public boolean isFinished() {
        return planner.planner.getPath().getLength() <= tolerance;
    }
}
