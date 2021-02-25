package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.Point;
import frc.robot.subsystems.MotionGenerationSubsystem;

public class MotionGenerationCommand extends CommandBase {
    public final DifferentialDriveFilterCommand filter;
    public final MotionGenerationSubsystem motion;
    private Pose2d prev;
    private long t0;
    private Point vel_translational = new Point(0, 0);
    private double vel_rotational = 0;

    public MotionGenerationCommand(DifferentialDriveFilterCommand filter, MotionGenerationSubsystem motion) {
        this.filter = filter;
        this.motion = motion;
        prev = filter.getFilteredPose();
        t0 = RobotController.getFPGATime();
    }

    @Override
    public void execute() {
        Pose2d pos = filter.getFilteredPose();
        Transform2d vel = pos.minus(prev);
        double t_diff = (RobotController.getFPGATime() - t0) / 1000000.;
        SimpleMatrix x = new SimpleMatrix(new double[][]{{pos.getX()}, {pos.getY()}, {pos.getRotation().getRadians()}});
        SimpleMatrix x_dot = new SimpleMatrix(new double[][]{{vel.getX() / t_diff}, {vel.getY() / t_diff}, {vel.getRotation().getRadians() / t_diff}});
        SimpleMatrix accel = motion.rootSolve(x, x_dot);
        long t1 = RobotController.getFPGATime();
        t_diff = (t1 - t0) / 1000000.;
        vel_translational = new Point(vel.getX() + accel.get(0) * t_diff, vel.getY() + accel.get(1) * t_diff);
        vel_rotational = vel.getRotation().getRadians() + accel.get(2) * t_diff;
        t0 = t1;
        prev = pos;
    }

    public Point getVelocity() {
        return vel_translational;
    }

    public double getRotationalVelocity() {
        return vel_rotational;
    }
}
