package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;
import frc.robot.mapping.obstacle.ObstacleReader;
import frc.robot.motion.generation.rmpflow.RMPRoot;
import frc.robot.motion.generation.rmpflow.rmps.ObstacleAvoidance;
import frc.robot.subsystems.DifferentialDriveSubsystem;

public class AssistedDriveCommand extends CommandBase {
    public final DifferentialDriveOdometryCommand filter;
    public final DifferentialDriveSubsystem drive;
    private final RMPRoot root = new RMPRoot("Assisted Driving");
    private long t0 = 0;
    private double lambda = 0;
    private SimpleMatrix prev = new SimpleMatrix(new double[][]{{0}, {0}, {0}});

    public AssistedDriveCommand(DifferentialDriveSubsystem drive, DifferentialDriveOdometryCommand filter, ObstacleAvoidance obstacleRMP, double lambda) {
        this.filter = filter;
        this.drive = drive;
        this.lambda = lambda;
        t0 = RobotController.getFPGATime();
        //root.linkChild(obstacleRMP);
        ObstacleAvoidance.ObstacleAvoidanceStatic.addRootObstacleChildren(ObstacleReader.readWithoutExceptions(), root, DifferentialDriveSubsystem.ROBOT_TRACK_WIDTH/2);
    }
    public AssistedDriveCommand(DifferentialDriveSubsystem drive, DifferentialDriveOdometryCommand filter, ObstacleAvoidance obstacleRMP) {
        this(drive, filter, obstacleRMP, 1.25);
    }

    private SimpleMatrix reduce(SimpleMatrix xyz) {
        return new SimpleMatrix(new double[][]{{xyz.get(0)}, {xyz.get(1)}});
    }
    private SimpleMatrix expand(SimpleMatrix xy) {
        return new SimpleMatrix(new double[][]{{xy.get(0)}, {xy.get(1)}, {0}});
    }

    @Override
    public void execute() {
        double target_left = XboxMap.left()*10;
        double target_right = XboxMap.right()*10;
        double speed = (target_left + target_right) / 2;
        //Pose2d pose = filter.getFilteredPose();
        Pose2d pose = filter.getPose();
        SimpleMatrix target = new SimpleMatrix(new double[][]{
            {speed * pose.getRotation().getCos()},
            {speed * pose.getRotation().getSin()},
            {(target_right - target_left) / DifferentialDriveSubsystem.ROBOT_TRACK_WIDTH}});
        SimpleMatrix x = new SimpleMatrix(
            new double[][]{{pose.getX()}, {pose.getY()}});
        SimpleMatrix acc = target.minus(prev).scale(lambda).plus(expand(root.solve(x, reduce(prev)))).scale(2);
        SmartDashboard.putNumber("Accelerator", Math.sqrt(acc.get(0) * acc.get(0) + acc.get(1) * acc.get(1)));
        //SimpleMatrix acc = new SimpleMatrix(new double[][]{{acc2d.get(0)}, {acc2d.get(1)}, {0}});
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
            {(right - left) * 10 / DifferentialDriveSubsystem.ROBOT_TRACK_WIDTH}
        });
        t0 = t1;
    }
}
