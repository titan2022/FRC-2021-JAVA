/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
/**
 * 
 */
public class LimelightSubsystem extends SubsystemBase {
  private static NetworkTable table;

  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight"); //initialize table
    table.getEntry("getPipe").setNumber(0); //set default pipeline
  }

  /**
   * 
   */
  public static void setPipeline(Constants.LimelightPipeline i)
  {
    table.getEntry("getPipe").setNumber(i.pipelineNum());
  }

  /**
   * 
   * @return
   */
  public static double getPipeline()
  {
    return table.getEntry("getPipe").getDouble(0);
  }

  /**
   * 
   * @return
   */
  public static boolean validTarget()
  {
    if(table.getEntry("tv").getDouble(0)==0)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  /**
   * 
   * @return
   */
  public static double getX()
  {
    return table.getEntry("tx").getDouble(0);
  }

  /**
   * 
   * @return
   */
  public static double getY()
  {
    return table.getEntry("ty").getDouble(0);
  }

  /**
   * 
   * @return
   */
  public static double[] getCamPose()
  {
    return table.getEntry("camtran").getDoubleArray(new double[]{});
  }

  /**
   * 
   * @return
   */
  public static double getLatency()
  {
    return table.getEntry("tl").getDouble(0);
  }
  
  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    // This method will be called once per scheduler run
  }
}
