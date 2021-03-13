package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motion.generation.rmpflow.RMPRoot;
import frc.robot.subsystems.DriveSubsystem;

public class RMPDrive extends CommandBase {
    public final RMPRoot root;
    public final DifferentialDriveFilterCommand filter;
    public final DriveSubsystem drive;
    private SimpleMatrix x0;
    private long t0;

    public RMPDrive(RMPRoot root, DriveSubsystem drive, DifferentialDriveFilterCommand filter) {
        this.root = root;
        this.drive = drive;
        this.filter = filter;
        t0 = RobotController.getFPGATime();
        x0 = toVector(filter.getFilteredPose());
        addRequirements(drive);
    }

    private SimpleMatrix toVector(Pose2d x) {
        return new SimpleMatrix(new double[][]{{x.getX()}, {x.getY()}, {x.getRotation().getRadians()}});
    }
    private ChassisSpeeds toIntrinsic(SimpleMatrix v, Rotation2d theta) {
        return new ChassisSpeeds(v.get(0) * theta.getCos() + v.get(1) * theta.getSin(),
            v.get(0) * theta.getSin() + v.get(1) * theta.getCos(),
            v.get(2));
    }

    @Override
    public void execute() {
        double dt = (RobotController.getFPGATime() - t0) / 1000.;
        SimpleMatrix x1 = toVector(filter.getFilteredPose());
        SimpleMatrix v0 = x1.minus(x0).scale(dt);
        SimpleMatrix acc = root.solve(x1, v0);
        SimpleMatrix v1 = acc.scale(dt).plus(v0);
        drive.setVelocities(toIntrinsic(v1, new Rotation2d(x1.get(2))));
        x0 = x1;
        t0 += dt * 1000;
    }
}
