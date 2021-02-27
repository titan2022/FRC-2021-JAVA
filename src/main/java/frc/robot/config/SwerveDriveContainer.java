package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.FieldDisplayCommand;
import frc.robot.commands.ManualSwerveDriveCommand;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class SwerveDriveContainer implements RobotContainer {
    // Subsystems
    private final SwerveDriveSubsystem swerveDriveSub;
    private final NavigationSubsystem navSub;

    // Command Groups
    private final ParallelCommandGroup autoGroup;
    private final ParallelCommandGroup teleopGroup;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public SwerveDriveContainer(boolean simulated) {
        // Initialize Subsystems
        swerveDriveSub = new SwerveDriveSubsystem(getLeftSwerveDriveTalonConfig(), getRightSwerveDriveTalonConfig());
        navSub = new NavigationSubsystem();

        // Initialize Auto Commands
        FieldDisplayCommand autoFieldDisplayCommand = new FieldDisplayCommand("Auto Field");

        // Initialize Teleop Commands
        FieldDisplayCommand fieldDisplayCommand = new FieldDisplayCommand();
        ManualSwerveDriveCommand manualDriveCommand = new ManualSwerveDriveCommand(swerveDriveSub, navSub);

        // Initialize Command Groups
        autoGroup = new ParallelCommandGroup(autoFieldDisplayCommand); // These don't actually run in parallel.
        teleopGroup = new ParallelCommandGroup(fieldDisplayCommand, manualDriveCommand);

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
    public TalonFXConfiguration getLeftSwerveDriveTalonConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:

        return talon;
    }

    /**
     * Method containing the talon configuration of the right side of the differential drive.
     * @return The talon configuration of the right side of the drive.
     */
    public TalonFXConfiguration getRightSwerveDriveTalonConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:

        return talon;
    }
}
