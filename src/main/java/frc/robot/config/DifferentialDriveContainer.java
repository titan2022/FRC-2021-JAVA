package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ManualDifferentialDriveCommand;
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
    private final NavigationSubsystem navigationSub;

    // Commands
    private final ManualDifferentialDriveCommand manualDrive;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public DifferentialDriveContainer(boolean isSimulated) {
        // Initialize Subsystems
        diffDriveSub = new DifferentialDriveSubsystem(getLeftDiffDriveTalonConfig(), getRightDiffDriveTalonConfig(), isSimulated);
        navigationSub = new NavigationSubsystem(diffDriveSub);

        // Initialize Commands
        manualDrive = new ManualDifferentialDriveCommand(diffDriveSub);

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
        return null;
    }

    @Override
    public Command getTeleopCommand() {
        return manualDrive;
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
}
