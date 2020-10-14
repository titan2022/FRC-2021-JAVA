/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
public class AutoAimSubsystem extends SubsystemBase {
  /**
   * Creates a new AutoAimSubsystem.
   */
  public static double distance;
  public static double angleToTarget;
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  public static double x;
  public static double y;
  public double steeringAdjust;
  public AutoAimSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    x = tx.getDouble(0.0);
    y= ty.getDouble(0.0);
  }

  public static void calculateDistance()
  {
    distance = (Constants.targetHeight - Constants.limelightHeight) / Math.tan(Constants.limelightAngle + angleToTarget);
  }
  public static void calculateAngleToTarget()
  {
    double nx = (1/160) *(x -159.5);  //normalized pixel values
    //double ny =(1/120) * (119.5 -y);
    double vpw = 2.0 *Math.tan(54/2);  //calculates horizontal fov
    //double vph = 2.0 *Math.tan(41/2);
    double xCoor = vpw/2 *nx;
    angleToTarget = Math.atan(xCoor/1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
   
  }
}
