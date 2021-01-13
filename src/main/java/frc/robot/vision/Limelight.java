/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
/**
 * Limelight class for returning distance and angle to reflective targets
 */
public class Limelight {

    public static double targetHeight = 1; //test values
    public static double limelightHeight= 1;
    public static double limelightAngle = 90;
    LimelightSubsystem subsystem = new LimelightSubsystem();

    public Limelight()
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);//Set to use vision camera
    }
 /**
   * Limelight Constructor
   * @return
   */
  public double calculateDistance()
  {
    double distance = (targetHeight - limelightHeight) / Math.tan(limelightAngle + calculateAngleToTargetV());
    return distance;
  }

  /**
   * Calculates straight line distance to target from position of camera
   * @return
   */
  public double calculateAngleToTargetV()
  {
    double nx = (1/160) *(subsystem.getX() -159.5);  //normalized pixel values
    //double ny =(1/120) * (119.5 -y);
    double vpw = 2.0 *Math.tan(54/2);  //calculates horizontal fov
    //double vph = 2.0 *Math.tan(41/2);
    double xCoor = vpw/2 *nx;
    double angleToTarget = Math.atan(xCoor/1);
    return angleToTarget;
  }
  /**
   * Calculates Vertical Angle to Target
   * @return
   */
  public double calculateAngleToTargetH()  
  {
    double ny =(1/120) * (119.5 -subsystem.getY());
    double vph = 2.0 *Math.tan(41/2);
    double yCoor = vph/2 *ny;
    double angleToTarget = Math.atan(yCoor/1);
    return angleToTarget;
  }
  /**
   * Calculates Horizontal Angle to Target
   * @return
   */
  
  public boolean relativeCamPosition() //TODO: Transform it to be relative to the camera.
  {
    return false;
  }

  public double latency()
  {
    return subsystem.getLatency();
  }
  public double[] camPos()
  {
    return subsystem.getCamPose();
  }
  public boolean validTarget()
  {
    return subsystem.validTarget();
  }
  public String pipeline()
  {
    return subsystem.getPipeline();
  }
  public void setPipeline(LimelightEnum i)
  {
    subsystem.setPipeline(i);
  }
   /**
   * Returns target position relative to camera
   * @return
   */

}
