/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.guidemos;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WidgetDemo extends CommandBase {
  /**
   * Creates a new LayoutDemo.
   */
  public WidgetDemo() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShuffleboardTab tab = Shuffleboard.getTab("Widget Demo Tab");  
    tab.add("Message","This is how to add a graph to a Layout:")
      .withWidget(BuiltInWidgets.kTextView);
    tab.add("Graph 1",122.2)
      .withWidget(BuiltInWidgets.kGraph);
    tab.add("Message","To create the graph, first create the widget using the add method from the ShuffleBoardTab class. Then use the withWidget method to set the what widget you want the value to appear in. For example, for the data, use 'tab.add('Title',111(this can be any value))'. Then to set the widget, use '.withWidget(BuiltinWidgets.kGraph);'")
      .withWidget(BuiltInWidgets.kTextView);    
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
