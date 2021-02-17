/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.motion.control.CustomKalmanFilter;
import com.kauailabs.navx.frc.AHRS;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Navigation Subsystem using Differential Drive Odometry and Gyro
 */
public class NavigationSubsystem extends SubsystemBase {
  // ALL ODOMETRY DONE IN METERS, not imperial
  private static final double STATE_STD_DEV = 0.1; // meters
  private static final double MEAS_STD_DEV = 0.01; // meters

  // Navigation Mathematics system
  private CustomKalmanFilter filter; // vector: [xpos, xvel, xacc, ypos, yvel, yacc]
  private DifferentialDriveOdometry odometry;
  private Timer timer;
  private boolean simulated;

  // Physical and Simulated Hardware
  private AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 50);
  private DifferentialDriveSubsystem drive;

  // Simulated components
  // AHRS SimDoubles
  private SimDouble yaw; // degs
  private SimDouble rate; // degs / sec
  private double simPrevT = 0;
  private double simPrevYaw = 0;

  // Physics simulation
  private Field2d fieldSim = new Field2d();

  /**
   * Creates a new NavigationSubsystem.
   * @param drive - Drive subsystem with primary motors.
   */
  public NavigationSubsystem(DifferentialDriveSubsystem drive) {
    this.simulated = drive.isSimulated();

    if (simulated) enableSimulation();

    this.drive = drive;
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    filter = new CustomKalmanFilter(new SimpleMatrix(6, 1), SimpleMatrix.identity(6),
        SimpleMatrix.identity(6).scale(Math.pow(STATE_STD_DEV, 2)),
        SimpleMatrix.identity(6).scale(Math.pow(MEAS_STD_DEV, 2)), updateA(0),
        new SimpleMatrix(new double[][] { { 0, 0 }, { 1, 0 }, { 0, 0 }, { 0, 0 }, { 0, 1 }, { 0, 0 } }),
        SimpleMatrix.identity(6));

    timer = new Timer();
    timer.start();
  }

  private void enableSimulation() {

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    yaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    rate = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Rate"));

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
   * Gets yaw computed by AHRS gyro.
   * 
   * @return Yaw (degrees).
   */
  public double getYaw() {

    // return gyro.getYaw();

    if (simulated) {

      return yaw.get();

    } else {

      return gyro.getYaw();

    }

  }

  /**
   * Gets yaw computed by driveSim.
   * 
   * @return Yaw (degrees).
   */
  private double getDriveSimYaw() {

    return drive.getDriveSim().getHeading().getDegrees();

  }

  /**
   * Gets heading computed by AHRS gyro.
   * 
   * @return Heading (degrees).
   */
  public double getHeading() {

    return Math.IEEEremainder(getYaw(), 360);

  }

  /**
   * Gets Rotation2d object representing heading computed by AHRS gyro.
   * 
   * @return Heading (Rotation2d object).
   */
  public Rotation2d getHeadingRotation() {

    return Rotation2d.fromDegrees(getHeading());

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


  // filter methods

  /**
   * Gets filterable vector motion measurement from odometry.
   * 
   * @return Odometry vector measurement.
   */
  private SimpleMatrix getOdometryMeas() {

    return new SimpleMatrix(new double[][] { { odometry.getPoseMeters().getX() }, { 0 }, { 0 },
        { odometry.getPoseMeters().getY() }, { 0 }, { 0 } });

  }

  /**
   * Gets filterable vector motion measurement from gyro.
   * 
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
   * @param t - Time between filter runs.
   * @return Updated A matrix.
   */
  private SimpleMatrix updateA(double t) {

    return new SimpleMatrix(new double[][] { { 1, t, Math.pow(t, 2) / 2, 0, 0, 0 }, { 0, 1, t, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 1, t, Math.pow(t, 2) / 2 }, { 0, 0, 0, 0, 1, t }, { 0, 0, 0, 0, 0, 0 } });

  }

  /**
   * Returns the Kalman filter's current state.
   * 
   * @return Current Kalman filter state.
   */
  public SimpleMatrix getFilterState() {

    return filter.getState();

  }

  /**
   * Returns an element from the Kalman filter's current state.
   * 
   * @param row    - Row of state.
   * @param column - Row of state.
   * @return Element from current filter state.
   */
  public double getFilterStateElement(int row, int column) {

    return filter.getState().get(row, column);

  }

  /**
   * Gets FPGA time from robot and converts it to seconds.
   * 
   * @return FPGA time in seconds.
   */
  public double getRobotTime() {

    return RobotController.getFPGATime() / 1e6;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    filter.setA(updateA(timer.get()));
    timer.reset();

    // TODO: add filter.predictFilter(input(velocity)), bringing input from Xbox controller
    filter.updateFilter(getOdometryMeas());
    filter.updateFilter(getGyroMeas());
  }

  public void simulationPeriodic() {
    yaw.set(getDriveSimYaw());

    rate.set((getDriveSimYaw() - simPrevYaw) / (getRobotTime() - simPrevT));
    simPrevT = getRobotTime();
    simPrevYaw = getDriveSimYaw();

    //fieldSim.setRobotPose(getFilterStateElement(0, 0), getFilterStateElement(3, 0), Rotation2d.fromDegrees(getHeading())); // TODO: Debug

    fieldSim.setRobotPose(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), Rotation2d.fromDegrees(getHeading()));

    SmartDashboard.putData("Field", fieldSim);
  }
}
