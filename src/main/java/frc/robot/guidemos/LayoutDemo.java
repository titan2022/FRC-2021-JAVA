/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.guidemos;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LayoutDemo extends CommandBase {
    /**
     * Creates a new LayoutDemo.
     */
    public LayoutDemo() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      ShuffleboardTab tab = Shuffleboard.getTab("Layout Demo Tab");
        tab.add("Message", "This is how to change the layout through code:")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0,0)
        .withSize(3, 1);
      tab.add("Message 2",
        "To align a widget in a specific position, you need to use the '.withPosition' method and you need to specify the x and y coordinates for where you would like to place the widget. For example, you could use .withWidget(10,11) to place the widget 10 units to the right and 11 units down.")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0,1)
        .withSize(5,1);
      tab.add("Widget 1", 122.2)
          .withPosition(3,0);     
      tab.add("Widget 2", 122.2)
        .withPosition(5,1);
      tab.add("Message 3",
        "To resize a widget, use the '.withSize' method. For example, in the two dials shown, for the first one I used '.withSize(5,5)' and for the second one I used '.withSize(10,10)'.")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0,2)
        .withSize(8,1);      
      tab.add("Widget 3", 150.5).withWidget(BuiltInWidgets.kDial)
        .withPosition(0,3)
        .withSize(1,1);
      tab.add("Widget 4", 150.5)
      .withWidget(BuiltInWidgets.kDial)
      .withPosition(1,3)
      .withSize(2,1);
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
