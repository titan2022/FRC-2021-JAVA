/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.motioncontrol.CustomKalmanFilter;
import com.kauailabs.navx.frc.AHRS;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Navigation Subsystem using Differential Drive Odometry and Gyro
 */
public class NavigationSubsystem extends SubsystemBase {

  // ALL ODOMETRY DONE IN METERS, not imperial

  private static final double STATE_STD_DEV = 0.1; // meters
  private static final double MEAS_STD_DEV = 0.01; // meters

  private AHRS gyro;
  private DifferentialDriveOdometry odometry;
  private DriveSubsystem drive;
  private CustomKalmanFilter filter; // vector: [xpos, xvel, xacc, ypos, yvel, yacc]
  private Timer timer;
  private double t;

  /**
   * Creates a new NavigationSubsystem.
   * 
   * @param drive - Drive subsystem with primary motors.
   */
  public NavigationSubsystem(DriveSubsystem drive) {

    gyro = new AHRS(SPI.Port.kMXP);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    this.drive = drive;

    filter = new CustomKalmanFilter(new SimpleMatrix(6, 1), SimpleMatrix.identity(6),
        SimpleMatrix.identity(6).scale(Math.pow(STATE_STD_DEV, 2)),
        SimpleMatrix.identity(6).scale(Math.pow(MEAS_STD_DEV, 2)), updateA(),
        new SimpleMatrix(new double[][] { { 0, 0 }, { 1, 0 }, { 0, 0 }, { 0, 0 }, { 0, 1 }, { 0, 0 } }),
        SimpleMatrix.identity(6));

    timer = new Timer();
    timer.start();

  }

  // gyro methods, copied from 2020

  /**
   * Gets the AHRS gyro in its current state.
   * 
   * @return Gyro.
   */
  public AHRS getGyro() {

    return gyro;

  }

  /**
   * Gets angle computed by AHRS gyro.
   * 
   * @return Angle (degrees).
   */
  public double getAngle() {

    return gyro.getAngle();

  }

  /**
   * Gets heading computed by AHRS gyro.
   * 
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
   * 
   * @return Differential drive odometry.
   */
  public DifferentialDriveOdometry getOdometry() {

    return odometry;

  }

  /**
   * Updates differential drive odometry.
   */
  public void updateOdometry() {

    odometry.update(Rotation2d.fromDegrees(getHeading()), drive.getEncoderDist(true), drive.getEncoderDist(false));

  }

  // filter methods

  /**
   * Gets filterable vector motion measurement from odometry.
   * @return Odometry vector measurement.
   */
  private SimpleMatrix getOdometryMeas() {

    return new SimpleMatrix(new double[][] { { odometry.getPoseMeters().getX() }, { 0 }, { 0 },
        { odometry.getPoseMeters().getY() }, { 0 }, { 0 } });

  }

  /**
   * Gets filterable vector motion measurement from gyro.
   * @return Gyro vector measurement.
   */
  private SimpleMatrix getGyroMeas() {

    return new SimpleMatrix(
        new double[][] { { gyro.getDisplacementX() }, { gyro.getVelocityX() }, { gyro.getWorldLinearAccelX() },
            { gyro.getDisplacementZ() }, { gyro.getVelocityZ() }, { gyro.getWorldLinearAccelZ() } });

  }

  /**
   * Updates A matrix for specific time.
   * 
   * @return Updated A matrix.
   */
  private SimpleMatrix updateA() {

    return new SimpleMatrix(new double[][] { { 1, t, Math.pow(t, 2) / 2, 0, 0, 0 }, { 0, 1, t, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 1, t, Math.pow(t, 2) / 2 }, { 0, 0, 0, 0, 1, t }, { 0, 0, 0, 0, 0, 0 } });

  }

  /**
   * Updates A matrix, and runs an iteration of the Kalman filter.
   * 
   * @param input - Control input from user.
   * @param meas - Measurement from sensor.
   */
  private void doKalmanFilter(SimpleMatrix input, SimpleMatrix meas) {

    t = timer.get();

    filter.setA(updateA());
    filter.runFilter(input, meas);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(Rotation2d.fromDegrees(getHeading()), drive.getEncoderDist(true), drive.getEncoderDist(false));
    doKalmanFilter(getOdometryMeas(), new SimpleMatrix(2, 1));
    doKalmanFilter(getGyroMeas(), new SimpleMatrix(2, 1));
    timer.reset();
    
  }


}
