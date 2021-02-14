package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class TurnToAngle extends CommandBase {
    public final double P;
    public final double I;
    public final double D;
    public final Rotation2d target;
    public final DriveSubsystem drive;
    public final NavigationSubsystem nav;
    public final double tolerance;
    private Rotation2d prev;
    private double sum = 0;

    public TurnToAngle(DriveSubsystem drive, Rotation2d target, double tolerance, NavigationSubsystem nav, double P, double I, double D) {
        this.drive = drive;
        this.target = target;
        this.nav = nav;
        this.P = P;
        this.I = I;
        this.D = D;
        this.tolerance = tolerance;
        addRequirements(drive);
    }
    public TurnToAngle(DriveSubsystem drive, Rotation2d target, double tolerance, NavigationSubsystem nav, double P, double I) {
        this(drive, target, tolerance, nav, P, I, 0);
    }
    public TurnToAngle(DriveSubsystem drive, Rotation2d target, double tolerance, NavigationSubsystem nav, double P) {
        this(drive, target, tolerance, nav, P, 0);
    }
    public TurnToAngle(DriveSubsystem drive, Rotation2d target, NavigationSubsystem nav, double P, double I, double D) {
        this(drive, target, 0, nav, P, I, D);
    }
    public TurnToAngle(DriveSubsystem drive, Rotation2d target, NavigationSubsystem nav, double P, double I) {
        this(drive, target, 0, nav, P, I);
    }
    public TurnToAngle(DriveSubsystem drive, Rotation2d target, NavigationSubsystem nav, double P) {
        this(drive, target, 0, nav, P);
    }

    @Override
    public void initialize() {
        prev = Rotation2d.fromDegrees(nav.getHeading()).minus(target);
    }

    @Override
    public void execute() {
        Rotation2d err = Rotation2d.fromDegrees(nav.getHeading()).minus(target);
        sum += err.getRadians();
        double derivative = prev.minus(err).getRadians();
        prev = err;
        drive.setVelocities(new ChassisSpeeds(0, 0, P*err.getRadians() + I*sum - D*derivative));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(prev.getRadians()) <= tolerance;
    }
}
