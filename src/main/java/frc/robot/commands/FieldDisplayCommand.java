package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FieldDisplayCommand extends CommandBase {

  private final Field2d fieldDisplay = new Field2d();
  private final String fieldDisplayName;

  /** 
   * Creates a new FieldDisplayCommand.
   */
  public FieldDisplayCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    fieldDisplayName = "Field";
  }

  /**
   * Creates a new FieldDisplayCommand with a field name.
   * @param fieldDisplayName - Field display name.
   */
  public FieldDisplayCommand(String fieldDisplayName) {
    this.fieldDisplayName = fieldDisplayName;
  }

  public void setRobotPose(Pose2d pose) {
    fieldDisplay.setRobotPose(pose);
  }

  public void setRobotPose(double xMeters, double yMeters, Rotation2d rotation) {
    fieldDisplay.setRobotPose(xMeters, yMeters, rotation);
  }

  public void setRobotPose(double xMeters, double yMeters, double degrees) {
    fieldDisplay.setRobotPose(xMeters, yMeters, Rotation2d.fromDegrees(degrees));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putData(fieldDisplayName, fieldDisplay);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putData(fieldDisplayName, fieldDisplay);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
