/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.subsystems.sim.PhysicsSim;

import com.kauailabs.navx.frc.AHRS;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Navigation Subsystem using AHRS Gyro
 */
public class NavigationSubsystem extends SubsystemBase {
  
  private boolean simulated;

  // Physical and Simulated Hardware
  private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 50);

  // Simulated components
  private DifferentialDriveSubsystem simulatedDriveSub;
  private SimDouble yaw; // degs
  private SimDouble rate; // degs / sec
  private double prevT;
  private double prevYaw;

  /**
   * Creates a new (non-simulated) NavigationSubsystem.
   */
  public NavigationSubsystem() {

    simulated = false;

  }

  // TEMPORARY CONSTRUCTOR to deal with simulation of navX-Sensor
  // TODO: Remove coupling between DifferentialDriveSubsystem and NavigationSubsystem
  /**
   * Creates a new (simulated) NavigationSubsystem.
   * @param simulatedDriveSub - Drive subsystem with primary motors.
   */
  public NavigationSubsystem(DifferentialDriveSubsystem simulatedDriveSub) {

    this();
    simulated = simulatedDriveSub.isSimulated();

    if (simulated) {
      
      this.simulatedDriveSub = simulatedDriveSub;

      int deviceHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      yaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(deviceHandle, "Yaw"));
      rate = new SimDouble(SimDeviceDataJNI.getSimValueHandle(deviceHandle, "Rate"));

      prevT = PhysicsSim.getFPGATime();
      prevYaw = simulatedDriveSub.getDriveSimYaw();

    }

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

    return (simulated) ? yaw.get() : gyro.getYaw();

  }

  /**
   * Gets yaw rate computed by AHRS gyro.
   * 
   * @return Yaw rate (degrees/sec).
   */
  public double getYawRate() {

    return (simulated) ? rate.get() : gyro.getRate();

  }

  /**
   * Gets heading computed by AHRS gyro.
   * 
   * @return Heading (degrees).
   */
  public double getHeadingDegrees() {

    return Math.IEEEremainder(getYaw(), 360);

  }

  /**
   * Gets Rotation2d object representing heading computed by AHRS gyro.
   * 
   * @return Heading (Rotation2d object).
   */
  public Rotation2d getHeadingRotation() {

    return Rotation2d.fromDegrees(getHeadingDegrees());

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
   * Gets filterable vector motion measurement from gyro.
   * 
   * @return Gyro vector measurement.
   */
  public SimpleMatrix getGyroVector() {

    return new SimpleMatrix(
        new double[][] { { gyro.getDisplacementX() }, { gyro.getVelocityX() }, { gyro.getWorldLinearAccelX() },
            { gyro.getDisplacementZ() }, { gyro.getVelocityZ() }, { gyro.getWorldLinearAccelZ() } });

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    
    yaw.set(simulatedDriveSub.getDriveSimYaw());
    rate.set((simulatedDriveSub.getDriveSimYaw() - prevYaw) / (PhysicsSim.getFPGATime() - prevT));

    prevT = PhysicsSim.getFPGATime();
    prevYaw = simulatedDriveSub.getDriveSimYaw();

  }
}
