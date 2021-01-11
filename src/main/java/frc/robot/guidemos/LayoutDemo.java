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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ShuffleboardTab tab = Shuffleboard.getTab("Grid Demo Tab");
        tab.add("Message", "This is how to change the layout through code:").withWidget(BuiltInWidgets.kTextView);
        tab.add("Widget 1", 122.2);
        tab.add("Widget 2", 135.75).withWidget(BuiltInWidgets.kNumberSlider).withPosition(10, 11);
        tab.add("Message",
                "To align a widget in a specific position, you need to use the '.withPosition' method and you need to specify the x and y coordinates for where you would like to place the widget. For example, you could use .withWidget(10,11) to place the widget 10 units to the right and 11 units down.")
                .withWidget(BuiltInWidgets.kTextView);
        tab.add("Widget 3", 150.5).withWidget(BuiltInWidgets.kDial).withPosition(20, 20).withSize(5, 5);
        tab.add("Widget 3", 150.5).withWidget(BuiltInWidgets.kDial).withPosition(30, 30).withSize(10, 10);
        tab.add("Message",
                "To resize a widget, use the '.withSize' method. For example, in the two dials shown, for the first one I used '.withSize(5,5)' and for the second one I used '.withSize(10,10)'.")
                .withWidget(BuiltInWidgets.kTextView);
        tab.add("Message", "Here's how to add items to a list.").withWidget(BuiltInWidgets.kTextView);
        ShuffleboardLayout List = tab.getLayout("List", BuiltInLayouts.kList)
            .withSize(4,4);
        List.add("Widget 1",122.2);
        List.add("Widget 2",135.75); 
        tab.add("Message","To add a list through code, first, create a list using the getLayout method and specify the layout as a list. For example: 'ShuffleboardLayout List = tab.getLayout('List', BuiltInLayouts.kList)'. Then, similar to how you would add a widget to a tab, add it to the list using the '.add' method.")
            .withWidget(BuiltInWidgets.kTextView);     
        tab.add("Message","Similarily, you can create a grid layout using 'BuiltInLayouts.kGrid'")
            .withWidget(BuiltInWidgets.kTextView);
        ShuffleboardLayout Grid = tab.getLayout("Grid", BuiltInLayouts.kGrid);
        Grid.add("Widgt 1",122.2);        

       
   


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
