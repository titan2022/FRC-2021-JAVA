package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.XboxMap;
import frc.robot.subsystems.DifferentialDriveSubsystem;

/**
 * 
 */
public class ManualDifferentialDriveCommand extends CommandBase {
  private static DifferentialDriveSubsystem DifferentialDriveSubsystem;
  private boolean brakeState = false;

  /** Creates a new ManualDifferentialDriveCommand. */
  public ManualDifferentialDriveCommand(DifferentialDriveSubsystem differentialDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(differentialDrive);
    DifferentialDriveSubsystem = differentialDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Manual Differential Drive Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    brakeState = XboxMap.toggleBrakes() ? !brakeState : brakeState;

    if (brakeState) {
      DifferentialDriveSubsystem.enableBrakes();
    }
    else { 
      DifferentialDriveSubsystem.disableBrakes();
      DifferentialDriveSubsystem.setOutput(ControlMode.PercentOutput, XboxMap.leftWheel(), XboxMap.rightWheel());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DifferentialDriveSubsystem.stop();
		System.out.println("Manual Differential Drive Command Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
