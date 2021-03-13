package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.Point;
import frc.robot.motion.generation.rmpflow.RMPRoot;
import frc.robot.motion.generation.rmpflow.rmps.Cartesian2d;
import frc.robot.motion.generation.rmpflow.rmps.GoalAttractor;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToGoal extends CommandBase {
    public final RMPRoot root;
    public final DifferentialDriveFilterCommand filter;
    public final DriveSubsystem drive;
    private final Cartesian2d coord;
    private SimpleMatrix x0;
    private long t0;

    public DriveToGoal(Point goal, RMPRoot root, DriveSubsystem drive, DifferentialDriveFilterCommand filter, String name) {
        this.root = root;
        coord = new Cartesian2d("DriveToGoal coordinates", root, 0, 1);
        new GoalAttractor(name, root, toVector(goal), 10, 1, 10, 1, 2, 2, 0.005);
        this.drive = drive;
        this.filter = filter;
        t0 = RobotController.getFPGATime();
        x0 = toVector(filter.getFilteredPose());
    }
    public DriveToGoal(Point goal, RMPRoot root, DriveSubsystem drive, DifferentialDriveFilterCommand filter) {
        this(goal, root, drive, filter, "DriveToGoal");
    }

    private SimpleMatrix toVector(Translation2d x) {
        return new SimpleMatrix(new double[][]{{x.getX()}, {x.getY()}});
    }
    private SimpleMatrix toVector(Pose2d x) {
        return new SimpleMatrix(new double[][]{{x.getX()}, {x.getY()}, {x.getRotation().getRadians()}});
    }
}
