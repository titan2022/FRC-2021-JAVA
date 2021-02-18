package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DifferentialDriveOdometryCommand;
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
      
    // Simulation Variables

    private Field2d fieldSim;
    
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
        
        // Initialize Simulation Variables

        if (simulated) fieldSim = new Field2d();

        // Initialize Subsystems

        diffDriveSub = new DifferentialDriveSubsystem(getLeftDiffDriveTalonConfig(), getRightDiffDriveTalonConfig(), simulated);
        navSub = new NavigationSubsystem(diffDriveSub);

        // Initialize Command Groups

        autoGroup = new ParallelCommandGroup(getDriveOdometry(simulated));
        teleopGroup = new ParallelCommandGroup(getDriveOdometry(simulated), getManualDrive());

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
     * Creates a manual differential drive command (configure here).
     * @return Manual differential drive command (configured).
     */
    private ManualDifferentialDriveCommand getManualDrive() {

        return new ManualDifferentialDriveCommand(diffDriveSub);

    }

    /**
     * Creates a differential drive odometry command (configure here).
     * @return Differential odometry drive command (configured).
     */
    private DifferentialDriveOdometryCommand getDriveOdometry(boolean simulated) {

        return (simulated) ? new DifferentialDriveOdometryCommand(diffDriveSub, navSub, fieldSim) : new DifferentialDriveOdometryCommand(diffDriveSub, navSub);

    }
}
