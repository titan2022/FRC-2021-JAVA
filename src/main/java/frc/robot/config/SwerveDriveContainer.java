package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.FieldDisplayCommand;
import frc.robot.commands.ManualSwerveDriveCommand;
import frc.robot.motion.control.PIDConfig;
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
        swerveDriveSub = new SwerveDriveSubsystem(getSwerveDriveTalonDirectionalConfig(), getSwerveDriveTalonRotaryConfig());
        navSub = new NavigationSubsystem();

        // Initialize Auto Commands
        FieldDisplayCommand autoFieldDisplayCommand = new FieldDisplayCommand("Auto Field");

        // Initialize Teleop Commands
        FieldDisplayCommand fieldDisplayCommand = new FieldDisplayCommand();
        ManualSwerveDriveCommand manualDriveCommand = new ManualSwerveDriveCommand(swerveDriveSub, navSub, getPIDConfig());

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
    public TalonFXConfiguration getSwerveDriveTalonDirectionalConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:
        talon.slot0.kP = 504.000000;
        talon.slot0.kI = 5.600000;
        talon.slot0.kD = 0.20000;        
        talon.slot0.kF = 19.300000;
        talon.slot0.integralZone = 900;
        talon.slot0.allowableClosedloopError = 217;
        talon.slot0.maxIntegralAccumulator = 254.000000;
        //talon.slot0.closedLoopPeakOutput = 0.869990;
        //talon.slot0.closedLoopPeriod = 33;
        talon.neutralDeadband = 0.199413;

        return talon;
    }

    public TalonFXConfiguration getSwerveDriveTalonRotaryConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:
        talon.slot0.kP = 504.000000;
        talon.slot0.kI = 5.600000;
        talon.slot0.kD = 0.20000;        
        talon.slot0.kF = 19.300000;
        talon.slot0.integralZone = 900;
        talon.slot0.allowableClosedloopError = 217;
        talon.slot0.maxIntegralAccumulator = 254.000000;
        //talon.slot0.closedLoopPeakOutput = 0.869990;
        //talon.slot0.closedLoopPeriod = 33;
        talon.neutralDeadband = 0.199413;

        return talon;
    }

    public PIDConfig getPIDConfig(){
        PIDConfig pidConfig = new PIDConfig();

        pidConfig.kP = 504.000000;
        pidConfig.kI = 5.600000;
        pidConfig.kD = 0.20000;
        
        pidConfig.CONTINOUS_MINIMUM = 0;
        pidConfig.CONTINOUS_MAXIMUM = 2*Math.PI;

        pidConfig.INTEGRATION_MIN = -0.5;
        pidConfig.INTEGRATION_MAX = 0.5;

        return pidConfig;
    }
}
