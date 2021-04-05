package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;
import frc.robot.motion.generation.rmpflow.RMPRoot;
import frc.robot.motion.generation.rmpflow.rmps.ObstacleAvoidance;
import frc.robot.subsystems.DifferentialDriveSubsystem;

public class AssistedDriveCommand extends CommandBase {
    public final DifferentialDriveFilterCommand filter;
    public final DifferentialDriveSubsystem drive;
    private final RMPRoot root = new RMPRoot("Assisted Driving");
    private long t0 = 0;
    private double lambda = 0;
    private SimpleMatrix prev = new SimpleMatrix(new double[][]{{0}, {0}, {0}});

    public AssistedDriveCommand(DifferentialDriveSubsystem drive, DifferentialDriveFilterCommand filter, ObstacleAvoidance obstacleRMP, double lambda) {
        this.filter = filter;
        this.drive = drive;
        this.lambda = lambda;
        t0 = RobotController.getFPGATime();
        root.linkChild(obstacleRMP);
    }
    public AssistedDriveCommand(DifferentialDriveSubsystem drive, DifferentialDriveFilterCommand filter, ObstacleAvoidance obstacleRMP) {
        this(drive, filter, obstacleRMP, 1);
    }

    @Override
    public void execute() {
        double target_left = XboxMap.leftWheel()*10;
        double target_right = XboxMap.rightWheel()*10;
        double speed = (target_left + target_right) / 2;
        Pose2d pose = filter.getFilteredPose();
        SimpleMatrix target = new SimpleMatrix(new double[][]{
            {speed * pose.getRotation().getCos()},
            {speed * pose.getRotation().getSin()},
            {(target_right - target_left) / DifferentialDriveSubsystem.ROBOT_TRACK_WIDTH}});
        SimpleMatrix x = new SimpleMatrix(
            new double[][]{{pose.getX()}, {pose.getY()}, {pose.getRotation().getRadians()}});
        SimpleMatrix acc = target.minus(prev).scale(lambda).plus(root.solve(x, prev));
        long t1 = RobotController.getFPGATime();
        double dt = (t1 - t0) / 1000000.;
        SimpleMatrix vel = acc.scale(dt).plus(prev);
        double fore = vel.get(0) * pose.getRotation().getCos() +
            vel.get(1) * pose.getRotation().getSin();
        double left = (fore - DifferentialDriveSubsystem.ROBOT_TRACK_WIDTH*vel.get(2)/2)/10;
        double right = (fore + DifferentialDriveSubsystem.ROBOT_TRACK_WIDTH*vel.get(2)/2)/10;
        double max_spd = Math.max(Math.abs(left), Math.abs(right));
        if(max_spd > 1){
            right /= max_spd;
            left /= max_spd;
        }
        drive.setOutput(ControlMode.PercentOutput, left, right);
        prev = new SimpleMatrix(new double[][]{
            {(left + right) * 5 * pose.getRotation().getCos()},
            {(left + right) * 5 * pose.getRotation().getSin()},
            {(right - left) * 10 / DifferentialDriveSubsystem.ROBOT_TRACK_WIDTH}});
        t0 = t1;
    }
}
