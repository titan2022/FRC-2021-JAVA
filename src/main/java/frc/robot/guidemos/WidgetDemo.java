/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.guidemos;

import java.util.Map;

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
    ShuffleboardTab tab = Shuffleboard.getTab("Widget Demo Tab");  
    tab.add("Message 1","To add a widget to you Shuffleboard, first call a tab using the getTab method. Then use the appropriate widgets from the BuiltInWidgets Class.")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0,0)
      .withSize(7,1);
    tab.add("Message 2","You can represent the same values through multiple widgets like sliders,number bars, and graphs")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0,1)
      .withSize(5,1);
    tab.add("Message 3","To create a graph, first create the widget using the add method from the ShuffleBoardTab class. Then use the withWidget method to set the value of the graph - '.withWidget(BuiltinWidgets.kGraph);'")
       .withWidget(BuiltInWidgets.kTextView)
       .withPosition(0,2)
       .withSize(6,1); 
    tab.add("Slider 1",2.5)
       .withWidget(BuiltInWidgets.kNumberSlider)
       .withPosition(6,3)
       .withProperties(Map.of("min",-2.5, "max", 2.5))
       .getEntry();
    tab.add("Message 4","To create a number slider, similarily use '.withWidget(BuiltinWidgets.kNumeberSlider);' and you can change the properties of the slider, such as minimum and maximum limits using the '.withProperties()' method")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0,3)
      .withSize(6,1);
    tab.add("Graph 1",122.2)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(6,1);
    
    
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
