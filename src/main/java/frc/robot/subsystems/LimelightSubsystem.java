/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.LimelightEnum;
/**
 * Limelight Subsystem class, contains getter and setter methods for NetworkTable values
 */
public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable table;
  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight"); //initialize table
    table.getEntry("getPipe").setNumber(0); //set default pipeline
  }

  /**
   * Changes pipeline based on enum value
   * @return
   */
  public void setPipeline(LimelightEnum i)
  {
    table.getEntry("getPipe").setNumber(i.ordinal());
  }

  
  /**
   * Returns current pipeline as string
   * @return
   */
  public String getPipeline()
  {
    int pipelineNumber = (int)table.getEntry("getPipe").getDouble(0);
    LimelightEnum pipeline = LimelightEnum.values()[pipelineNumber];
    return pipeline.name();
  }

  /**
   *  Checks for valid reflective target
   * @return
   */
  public boolean validTarget()
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
   * Returns X distance to reflective target
   * @return
   */
  public double getX()
  {
    return table.getEntry("tx").getDouble(0);
  }


   /**
   * Returns Y distance to reflective target
   * @return
   */
  public double getY()
  {
    return table.getEntry("ty").getDouble(0);
  }

 
   /**
   * Returns 3D cam pose relative to target
   * @return
   */
  public double[] getCamPose()
  {
    return table.getEntry("camtran").getDoubleArray(new double[]{});
  }

 
  /**
   * Returns current camera latency
   * @return
   */
  public double getLatency()
  {
    return table.getEntry("tl").getDouble(0);
  }
    
  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    // This method will be called once per scheduler run
  }
}