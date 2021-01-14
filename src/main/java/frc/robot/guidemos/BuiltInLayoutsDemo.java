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

public class BuiltInLayoutsDemo extends CommandBase {
  /**
   * Creates a new LayoutDemo.
   */
  public BuiltInLayoutsDemo() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      ShuffleboardTab tab = Shuffleboard.getTab("Built In Layouts Demo Tab");
      tab.add("Message", "This is how to change the layout through code:")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0,0)
      .withSize(3, 1);       
     ShuffleboardLayout list = tab.getLayout("List", BuiltInLayouts.kList);
     list.withPosition(0, 2);
     list.withSize(1,2);
     list.add("Widget 1", 122.2);
     list.add("Widget 2", 135.75);
     tab.add("Message 2","To add a list through code, first, create a list using the getLayout method and specify the layout as a list. For example: 'ShuffleboardLayout List = tab.getLayout('List', BuiltInLayouts.kList)'. Then, similar to how you would add a widget to a tab, add it to the list using the '.add' method.")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 1)
      .withSize(8,1);
    ShuffleboardLayout grid = tab.getLayout("Grid",BuiltInLayouts.kGrid);
    grid.withPosition(3,2);
    grid.withSize(2,2);
    grid.add("Widget 1",122.2);

       /*   
        tab.add("Message 6","Similarily, you can create a grid layout using 'BuiltInLayouts.kGrid'")
            .withWidget(BuiltInWidgets.kTextView);
        ShuffleboardLayout Grid = tab.getLayout("Grid", BuiltInLayouts.kGrid);
        Grid.add("Widget 1",122.2);*/
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
