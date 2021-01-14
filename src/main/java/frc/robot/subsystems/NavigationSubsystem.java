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

/**
 * Navigation Subsystem using Differential Drive Odometry and Gyro
 */
public class NavigationSubsystem extends SubsystemBase {

  // ALL ODOMETRY DONE IN METERS, not imperial

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

  /**
   * Gets the AHRS gyro in its current state.
   * @return Gyro.
   */
  public AHRS getGyro() {

    return gyro;

  }
  
  /**
   * Gets angle computed by AHRS gyro.
   * @return Angle (degrees).
   */
  public double getAngle() {

    return gyro.getAngle();

  }

  /**
   * Gets heading computed by AHRS gyro.
   * @return Heading (degrees).
   */
  public double getHeading() {

    return -Math.IEEEremainder(getAngle(), 360);

  }

  /**
   * Resets AHRS gyro.
   */
  public void resetGyro() {

    gyro.reset();

  }

  /**
   * Calibrates AHRS gyro.
   */
  public void calibrateGyro() {

    gyro.calibrate();

  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getEncoderCount(boolean useLeft) {

    if (useLeft) {

      return leftPrimary.getSelectedSensorPosition(0);
    
    } else {

      return rightPrimary.getSelectedSensorPosition(0);

    }

  }

  /**
   * Gets the distance from a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Distance from specified primary motor.
   */
  public double getEncoderDist(boolean useLeft) {

    return getEncoderCount(useLeft) * METERS_PER_TICK;

  }
  
  /**
   * Gets current velocity of a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Current velocity of a specified primary motor.
   */
  public double getEncoderVelocity(boolean useLeft) {

    if (useLeft) {

      return leftPrimary.getSelectedSensorVelocity(0) * METERS_PER_TICK;
    
    } else {

      return rightPrimary.getSelectedSensorVelocity(0) * METERS_PER_TICK;

    }

  }

  // odometry methods

  /**
   * Gets current DifferentialDriveOdometry.
   * @return Differential drive odometry.
   */
  public DifferentialDriveOdometry getOdometry() {

    return odometry;

  }

  /**
   * Updates differential drive odometry.
   */
  public void updateOdometry() {

    odometry.update(Rotation2d.fromDegrees(getHeading()), getEncoderDist(true), getEncoderDist(false));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
