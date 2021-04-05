package frc.robot.config;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutoNavChallenge;
import frc.robot.commands.DifferentialDriveFilterCommand;
import frc.robot.commands.DifferentialDriveOdometryCommand;
import frc.robot.commands.FieldDisplayCommand;
import frc.robot.commands.ManualDifferentialDriveCommand;
import frc.robot.commands.RMPDrive;
import frc.robot.motion.generation.rmpflow.RMPRoot;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

/** Add your docs here. */
public class AutonavContainer implements RobotContainer {
    // Subsystems
    private final DifferentialDriveSubsystem diffDriveSub;
    private final NavigationSubsystem navSub;
    private final static double INCH_TO_METER = 0.0254;

    // Command Groups
    private final ParallelCommandGroup autoGroup;
    private final ParallelCommandGroup teleopGroup;

    public AutonavContainer(boolean simulated, Translation2d... waypoints) {
        diffDriveSub = new DifferentialDriveSubsystem(getLeftDiffDriveTalonConfig(), getRightDiffDriveTalonConfig(), simulated);
        navSub = new NavigationSubsystem(diffDriveSub);

        FieldDisplayCommand fieldDisplayCommand = new FieldDisplayCommand();
        DifferentialDriveOdometryCommand odometryCommand = new DifferentialDriveOdometryCommand(diffDriveSub, navSub, fieldDisplayCommand);
        odometryCommand.resetOdometry(new Pose2d(0.8, 2.3, new Rotation2d(0)), new Rotation2d(0));
        DifferentialDriveFilterCommand filterCommand = new DifferentialDriveFilterCommand(odometryCommand, navSub);
        RMPRoot root = new RMPRoot("Autonav Challenge");
        RMPDrive rmpDrive = new RMPDrive(root, diffDriveSub, filterCommand);
        AutoNavChallenge autoNavChallenge = new AutoNavChallenge(rmpDrive, waypoints);

        autoGroup = new ParallelCommandGroup(fieldDisplayCommand, odometryCommand, filterCommand, rmpDrive, autoNavChallenge);

        FieldDisplayCommand debugFieldDisplayCommand = new FieldDisplayCommand();
        DifferentialDriveOdometryCommand debugOdometryCommand = new DifferentialDriveOdometryCommand(diffDriveSub, navSub, debugFieldDisplayCommand);
        debugOdometryCommand.resetOdometry(new Pose2d(0.8, 2.3, new Rotation2d(0)), new Rotation2d(0));
        DifferentialDriveFilterCommand debugFilterCommand = new DifferentialDriveFilterCommand(debugOdometryCommand, navSub);
        ManualDifferentialDriveCommand debugManualDriveCommand = new ManualDifferentialDriveCommand(diffDriveSub);

        teleopGroup = new ParallelCommandGroup(debugFieldDisplayCommand, debugOdometryCommand, debugFilterCommand, debugManualDriveCommand);
        configureButtonBindings();
    }

    public AutonavContainer(boolean simulated, String challengeName) {
        this(simulated, getWaypoints(challengeName));
    }

    private static Translation2d[] getWaypoints(String challengeName) {
        ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
        switch (challengeName) {
            case "barrel":
                waypoints.add(new Translation2d(150, 60));
                waypoints.add(new Translation2d(240, 120));
                waypoints.add(new Translation2d(300, 60));
                waypoints.add(new Translation2d(60, 100));
                break;
            default:
                waypoints.add(new Translation2d(150, 60));
                break;
        }
        int waypointListSize = waypoints.size();
        for (int i = 0; i < waypointListSize; i++) {
            waypoints.set(i, waypoints.get(i).times(INCH_TO_METER));
        }
        return waypoints.toArray(new Translation2d[waypointListSize]);
    }


    @Override
    public Command getAutonomousCommand() {
        return autoGroup;
    }

    @Override
    public Command getTeleopCommand() {
        return teleopGroup;
    }

        /**
     * Method containing the talon configuration of the left side of the differential drive.
     * @return The talon configuration of the left side of the drive.
     */
    public TalonSRXConfiguration getLeftDiffDriveTalonConfig()
    {
        TalonSRXConfiguration talon = new TalonSRXConfiguration();
        // Add configs here:
        talon.slot0.kP = 1;
        talon.slot0.kI = 0;
        talon.slot0.kD = 0;        
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 900;
        talon.slot0.allowableClosedloopError = 217;
        talon.slot0.maxIntegralAccumulator = 254.000000;
        //talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the PID controller
        //talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the PID controller

        return talon;
    }

    /**
     * Method containing the talon configuration of the right side of the differential drive.
     * @return The talon configuration of the right side of the drive.
     */
    public TalonSRXConfiguration getRightDiffDriveTalonConfig()
    {
        TalonSRXConfiguration talon = new TalonSRXConfiguration();
        // Add configs here:
        talon.slot0.kP = 1;
        talon.slot0.kI = 0;
        talon.slot0.kD = 0;        
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 900;
        talon.slot0.allowableClosedloopError = 217;
        talon.slot0.maxIntegralAccumulator = 254.000000;
        //talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the PID controller
        //talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the PID controller

        return talon;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {

    }
}