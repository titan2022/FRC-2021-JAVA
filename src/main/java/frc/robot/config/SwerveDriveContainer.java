package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.FieldDisplayCommand;
import frc.robot.commands.ManualSwerveDriveCommand;
import frc.robot.commands.ManualVHopperCommand;
import frc.robot.commands.ManualWristCommand;
import frc.robot.motion.control.PIDConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VHopperSubsystem;

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
    private final VHopperSubsystem vhopperSub;
    private final IntakeSubsystem intakeSub;

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
        vhopperSub = new VHopperSubsystem();
        intakeSub = new IntakeSubsystem();

        // Initialize Auto Commands
        FieldDisplayCommand autoFieldDisplayCommand = new FieldDisplayCommand("Auto Field");

        // Initialize Teleop Commands
        ManualSwerveDriveCommand manualDriveCommand = new ManualSwerveDriveCommand(swerveDriveSub, navSub, getSwerveHeadingPIDConfig());
        ManualVHopperCommand vhopperCommand = new ManualVHopperCommand(vhopperSub);
        ManualWristCommand intakeCommand = new ManualWristCommand(intakeSub);

        // Initialize Command Groups
        autoGroup = new ParallelCommandGroup(autoFieldDisplayCommand); // These don't actually run in parallel.
        teleopGroup = new ParallelCommandGroup(manualDriveCommand, vhopperCommand, intakeCommand);

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
     * Contains a velocity based PID configuration.
     * @return TalonFX Configuration Object
     */
    public TalonFXConfiguration getSwerveDriveTalonDirectionalConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
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
     * Contains a position based PID configuration
     * @return TalonFX Configuration Object
     */
    public TalonFXConfiguration getSwerveDriveTalonRotaryConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
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
     * The swerve field oriented heading PID configuration
     * @return PID configuration for {@link PIDController}
     */
    public PIDConfig getSwerveHeadingPIDConfig(){
        PIDConfig pidConfig = new PIDConfig();

        pidConfig.kP = 1;//504.000000;
        pidConfig.kI = 0;//5.600000;
        pidConfig.kD = 0;//0.20000;
        
        pidConfig.CONTINOUS_MINIMUM = 0;
        pidConfig.CONTINOUS_MAXIMUM = 2 * Math.PI;

        pidConfig.INTEGRATION_MIN = -0.5;
        pidConfig.INTEGRATION_MAX = 0.5;

        return pidConfig;
    }
}
