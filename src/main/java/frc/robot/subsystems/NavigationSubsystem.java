/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.numbers.N6;
import edu.wpi.first.wpiutil.math.numbers.N2;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Navigation Subsystem using Differential Drive Odometry and Gyro
 */
public class NavigationSubsystem extends SubsystemBase {

  // ALL ODOMETRY DONE IN METERS, not imperial
  
  private AHRS gyro;
  private DifferentialDriveOdometry odometry;
  private KalmanFilter<N6, N6, N2> filter;

  /**
   * Creates a new NavigationSubsystem.
   */
  public NavigationSubsystem() {

    gyro = new AHRS(SPI.Port.kMXP);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

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
  public void updateOdometry(DriveSubsystem drive) {

    odometry.update(Rotation2d.fromDegrees(getHeading()), drive.getEncoderDist(true), drive.getEncoderDist(false));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
