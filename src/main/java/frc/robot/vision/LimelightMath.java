/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
/**
 * Add your docs here.
 */
public class LimelightMath {

    private LimelightSubsystem subsystem;

    public LimelightMath()
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);//Set to use vision camera
        subsystem = new LimelightSubsystem();
    }
 /**
   * 
   * @return
   */
  public double calculateDistance()
  {
    double distance = (Constants.targetHeight - Constants.limelightHeight) / Math.tan(Constants.limelightAngle + calculateAngleToTarget());
    return distance;
  }

  /**
   * 
   * @return
   */
  public double calculateAngleToTarget()
  {
    double nx = (1/160) *(LimelightSubsystem.getX() -159.5);  //normalized pixel values
    //double ny =(1/120) * (119.5 -y);
    double vpw = 2.0 *Math.tan(54/2);  //calculates horizontal fov
    //double vph = 2.0 *Math.tan(41/2);
    double xCoor = vpw/2 *nx;
    double angleToTarget = Math.atan(xCoor/1);
    return angleToTarget;
  }
  
  /**
   * 
   */
  public boolean relativeCamPosition() //TODO: Transform it to be relative to the camera.
  {
    return false;
  }

  public void checkDistance() {
    if(LimelightSubsystem.validTarget())
    {
      calculateDistance();
    }
    else
    {
      DriverStation.reportError("Valid target not found", false);
    }
  }
}
