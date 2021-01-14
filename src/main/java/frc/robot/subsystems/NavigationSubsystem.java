/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class NavigationSubsystem extends SubsystemBase {

  public static final double METERS_PER_TICK = 0.1;
  
  private AHRS gyro;
  private DifferentialDriveOdometry odometry;
  private TalonSRX leftPrimary, rightPrimary;

  
  /**
   * Creates a new NavigationSubsystem.
   */
  public NavigationSubsystem() {

    gyro = new AHRS(SPI.Port.kMXP);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    leftPrimary = new TalonSRX(DriveSubsystem.LEFT_PRIMARY_PORT);
    rightPrimary = new TalonSRX(DriveSubsystem.RIGHT_PRIMARY_PORT);

  }

  // gyro methods, copied from 2020

  public double getAngle() {

    return gyro.getAngle();

  }

  public double getHeading() {

    return -Math.IEEEremainder(getAngle(), 360);

  }

  public void resetGyro() {

    gyro.reset();

  }

  // odometry methods

  public double getEncoderCount(boolean useLeft) {

    if (useLeft) {

      return leftPrimary.getSelectedSensorPosition(0);
    
    } else {

      return rightPrimary.getSelectedSensorPosition(0);

    }

  }

  public double getEncoderDist(boolean useLeft) {

    return getEncoderCount(useLeft) * METERS_PER_TICK;

  }
  

  public double getEncoderVelocity(boolean useLeft) {

    if (useLeft) {

      return leftPrimary.getSelectedSensorVelocity(0) * METERS_PER_TICK;
    
    } else {

      return rightPrimary.getSelectedSensorVelocity(0) * METERS_PER_TICK;

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
