package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;
import frc.robot.motion.generation.rmpflow.RMPRoot;
import frc.robot.motion.generation.rmpflow.rmps.ObstacleAvoidance;
import frc.robot.subsystems.DifferentialDriveSubsystem;

public class AssistedDriveCommand extends CommandBase {
    public final DifferentialDriveFilterCommand filter;
    public final DifferentialDriveSubsystem drive;
    private final DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(DifferentialDriveSubsystem.ROBOT_TRACK_WIDTH);
    private final RMPRoot root = new RMPRoot("Assisted Driving");
    private long t0 = 0;

    public AssistedDriveCommand(DifferentialDriveSubsystem drive, DifferentialDriveFilterCommand filter, ObstacleAvoidance obstacleRMP) {
        this.filter = filter;
        this.drive = drive;
        t0 = RobotController.getFPGATime();
        root.linkChild(obstacleRMP);
    }

    @Override
    public void execute() {
        DifferentialDriveWheelSpeeds ipt = new DifferentialDriveWheelSpeeds(XboxMap.left()*10, XboxMap.right()*10);
        ChassisSpeeds ipt_vel = kinematics.toChassisSpeeds(ipt);
        Pose2d pose = filter.getFilteredPose();
        double vx0 = ipt_vel.vxMetersPerSecond * pose.getRotation().getCos() +
            ipt_vel.vyMetersPerSecond * pose.getRotation().getSin();
        double vy0 = ipt_vel.vxMetersPerSecond * pose.getRotation().getCos() +
            ipt_vel.vyMetersPerSecond * pose.getRotation().getSin();
        SimpleMatrix x = new SimpleMatrix(new double[][]{{pose.getX()}, {pose.getY()}, {pose.getRotation().getRadians()}});
        SimpleMatrix x_dot = new SimpleMatrix(new double[][]{{vx0}, {vy0}, {ipt_vel.omegaRadiansPerSecond}});
        SimpleMatrix acc = root.solve(x, x_dot);
        long t1 = RobotController.getFPGATime();
        double dt = (t1-t0) / 1000000;
        ChassisSpeeds out_vel = new ChassisSpeeds(
            acc.get(0)*dt + vx0,
            acc.get(1)*dt + vy0,
            acc.get(2)*dt + ipt_vel.omegaRadiansPerSecond);
        DifferentialDriveWheelSpeeds out = kinematics.toWheelSpeeds(out_vel);
        double out_left = Math.min(1, Math.max(-1, out.leftMetersPerSecond/10));
        double out_right = Math.min(1, Math.max(-1, out.rightMetersPerSecond/10));
        drive.setOutput(ControlMode.PercentOutput, out_left, out_right);
        t0 = t1;
    }
}
