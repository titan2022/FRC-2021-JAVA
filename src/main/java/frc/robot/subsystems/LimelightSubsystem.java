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
public class LimelightSubsystem extends SubsystemBase {
  /**
   * Creates a new AutoAimSubsystem.
   */
  public static double distance;
  public static double angleToTarget;
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  public static double x;
  public static double y;
  public double steeringAdjust;
  public LimelightSubsystem() {
    
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
  public static double steering()
  {
    double heading_error = -x;
    double steeringAdjust = 0;
    if(x> 1.0)
    {
       steeringAdjust = Constants.KpAim* heading_error - Constants.min_command;
    }
    else if(LimelightSubsystem.x< 1.0)
    {
        steeringAdjust =  Constants.KpAim* heading_error + Constants.min_command;
    }
    return steeringAdjust;
  }
  public static double distance()
  {
    calculateAngleToTarget();
    double distanceError = Constants.desiredDistance - LimelightSubsystem.distance;
    return distanceError;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    x = tx.getDouble(0.0);
    y= ty.getDouble(0.0);
  }
}
