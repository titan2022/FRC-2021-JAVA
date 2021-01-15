/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N6;

import com.kauailabs.navx.frc.AHRS;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Navigation Subsystem using Differential Drive Odometry and Gyro
 */
public class NavigationSubsystem extends SubsystemBase {

  // ALL ODOMETRY DONE IN METERS, not imperial

  private static final double PERIODIC_UPDATE_TIME = 0.02; // seconds
  private static final double STATE_STD_DEV = 0.1; // meters
  private static final double MEAS_STD_DEV = 0.01; // meters

  private AHRS gyro;
  private DifferentialDriveOdometry odometry;
  private DriveSubsystem drive;
  private KalmanFilter<N6, N2, N6> filter; // vector: [xpos, xvel, xacc, ypos, yvel, yacc]

  /**
   * Creates a new NavigationSubsystem.
   * 
   * @param drive - Drive subsystem with primary motors.
   */
  public NavigationSubsystem(DriveSubsystem drive) {

    gyro = new AHRS(SPI.Port.kMXP);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    this.drive = drive;

    // copied A matrix from:
    // https://github.com/titan2022/FRC-2021-JAVA/blob/kalman-filter/src/main/java/frc/robot/motioncontrol/kalmanfilter/test/KalmanFilterTestCommand.java

    LinearSystem<N6, N2, N6> drivePlant = new LinearSystem<>(
        new Matrix<N6, N6>(new SimpleMatrix(
            new double[][] { { 1, PERIODIC_UPDATE_TIME, Math.pow(PERIODIC_UPDATE_TIME, 2) / 2, 0, 0, 0 },
                { 0, 1, PERIODIC_UPDATE_TIME, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 1, PERIODIC_UPDATE_TIME, Math.pow(PERIODIC_UPDATE_TIME, 2) / 2 },
                { 0, 0, 0, 0, 1, PERIODIC_UPDATE_TIME }, { 0, 0, 0, 0, 0, 0 } })),
        new Matrix<N6, N2>(
            new SimpleMatrix(new double[][] { { 0, 0 }, { 1, 0 }, { 0, 0 }, { 0, 0 }, { 0, 1 }, { 0, 0 } })),
        new Matrix<N6, N6>(SimpleMatrix.identity(6)), new Matrix<N6, N2>(new SimpleMatrix(6, 2)));

    filter = new KalmanFilter<>(Nat.N6(), Nat.N2(), drivePlant,
        ((Matrix<N6, N1>) VecBuilder.fill(STATE_STD_DEV, STATE_STD_DEV, STATE_STD_DEV, STATE_STD_DEV, STATE_STD_DEV, STATE_STD_DEV)),
        ((Matrix<N2, N1>) VecBuilder.fill(MEAS_STD_DEV, MEAS_STD_DEV)), PERIODIC_UPDATE_TIME);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
