/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Bridge Limit", true);
    SmartDashboard.putNumber("Bridge Angle",12);
    SmartDashboard.putNumber("Swerve Angle",81);
    SmartDashboard.putNumber("Left Drive Encoder", 92);
    SmartDashboard.putNumber("Right Drive Encoder", 15);
    SmartDashboard.putNumber("Turret Pot", 13);
    SmartDashboard.putNumber("Turret Pot Voltage", 12);
    SmartDashboard.putNumber("RPM", 1);
    SmartDashboard.getNumber("RPM",0);
    Shuffleboard.getTab("Tab 3")
      .add("Pi",3.14)
      .withWidget(BuiltInWidgets.kNumberSlider);
    Shuffleboard.getTab("Driver Tab")
      .addPersistent("Max Speed", 2.25)
      .withPosition(7,9);
    Shuffleboard.getTab("Driver Tab")
      .addPersistent("Length", 41)
      .withPosition(7,9);
    Shuffleboard.getTab("Drive")
      .add("Max Speed", 1)
      .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
      .getEntry();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
      
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
