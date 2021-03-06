package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AssistedDriveCommand;
import frc.robot.commands.DifferentialDriveFilterCommand;
import frc.robot.commands.DifferentialDriveOdometryCommand;
import frc.robot.commands.FieldDisplayCommand;
import frc.robot.commands.ManualDifferentialDriveCommand;
import frc.robot.mapping.obstacle.ObstacleReader;
import frc.robot.motion.generation.rmpflow.rmps.ObstacleAvoidance;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class DifferentialDriveContainer implements RobotContainer {
    // Subsystems
    private final DifferentialDriveSubsystem diffDriveSub;
    private final NavigationSubsystem navSub;

    // Command Groups
    private final ParallelCommandGroup autoGroup;
    private final ParallelCommandGroup teleopGroup;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public DifferentialDriveContainer(boolean simulated) {
        // Initialize Subsystems
        diffDriveSub = new DifferentialDriveSubsystem(getLeftDiffDriveTalonConfig(), getRightDiffDriveTalonConfig(), simulated);
        navSub = new NavigationSubsystem(diffDriveSub);

        // Initialize Auto Commands
        FieldDisplayCommand autoFieldDisplayCommand = new FieldDisplayCommand("Auto Field");
        DifferentialDriveOdometryCommand autoOdometryCommand = new DifferentialDriveOdometryCommand(diffDriveSub, navSub, autoFieldDisplayCommand);
        autoOdometryCommand.resetOdometry(new Pose2d(3, 3, new Rotation2d(0)), new Rotation2d(0)); // Starting Position
        DifferentialDriveFilterCommand autoFilterCommand = new DifferentialDriveFilterCommand(autoOdometryCommand, navSub);

        // Initialize Teleop Commands
        FieldDisplayCommand fieldDisplayCommand = new FieldDisplayCommand();
        DifferentialDriveOdometryCommand odometryCommand = new DifferentialDriveOdometryCommand(diffDriveSub, navSub, fieldDisplayCommand);
        odometryCommand.resetOdometry(new Pose2d(3, 3, new Rotation2d(0)), new Rotation2d(0)); // Starting Position
        DifferentialDriveFilterCommand filterCommand = new DifferentialDriveFilterCommand(odometryCommand, navSub);
        ManualDifferentialDriveCommand manualDriveCommand = new ManualDifferentialDriveCommand(diffDriveSub);
        AssistedDriveCommand assistDriveCommand = new AssistedDriveCommand(diffDriveSub, odometryCommand, getObstacleMapRMP(diffDriveSub.ROBOT_TRACK_WIDTH));

        // Initialize Command Groups
        autoGroup = new ParallelCommandGroup(autoFieldDisplayCommand
                                            , autoOdometryCommand
                                            , autoFilterCommand
                                            ); // These don't actually run in parallel.
        teleopGroup = new ParallelCommandGroup(fieldDisplayCommand
                                            , odometryCommand
                                            //, filterCommand
                                            //, manualDriveCommand
                                            , assistDriveCommand
                                            );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

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

        return talon;
    }

    /**
     * Infinite Recharge Field RMP for Field Avoidance
     * @param obstacleGrowthRadius
     * @return
     */
    public ObstacleAvoidance getObstacleMapRMP(double obstacleGrowthRadius)
    {
        return new ObstacleAvoidance("Infinite Recharge Field", ObstacleReader.readWithoutExceptions(), null, obstacleGrowthRadius);
    }
}
