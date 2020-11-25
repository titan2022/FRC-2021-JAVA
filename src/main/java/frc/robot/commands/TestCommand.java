/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/**
 * Add your docs here.
 */
public class TestCommand extends edu.wpi.first.wpilibj2.command.InstantCommand {
  /**
   * Add your docs here.
   */
  public TestCommand() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  public void initialize() {
    ShuffleboardTab tab = Shuffleboard.getTab("New Tab");
    tab.add("Pi", 3.14);
  }

}
