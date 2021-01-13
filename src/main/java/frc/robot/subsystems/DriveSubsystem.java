/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// using WPILib's docs' example from:
// https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html

public class DriveSubsystem extends SubsystemBase {

  // port numbers to be added later
  // maybe add constants to an enum?

  private final static int LEFT_PRIMARY_PORT = 1;
  private final static int LEFT_SECONDARY_PORT = 2;
  private final static int RIGHT_PRIMARY_PORT = 3;
  private final static int RIGHT_SECONDARY_PORT = 4;

  private TalonSRX leftPrimary, leftSecondary, rightPrimary, rightSecondary;

  private boolean inverted;
  private double driveSpeed;

  // encoders to be added in navigation?

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    // motor initialization block

    leftPrimary = new TalonSRX(LEFT_PRIMARY_PORT);
    leftSecondary = new TalonSRX(LEFT_SECONDARY_PORT);
    rightPrimary = new TalonSRX(RIGHT_PRIMARY_PORT);
    rightSecondary = new TalonSRX(RIGHT_SECONDARY_PORT);

    inverted = false;
    driveSpeed = 1;
    
    leftSecondary.follow(leftPrimary);
    rightSecondary.follow(rightPrimary);

    rightPrimary.setInverted(true);
    rightSecondary.setInverted(true);

    leftPrimary.setSensorPhase(true);
    rightPrimary.setSensorPhase(true);

    // motor config block

    TalonSRXConfiguration leftPrimaryConfig = new TalonSRXConfiguration();
    TalonSRXConfiguration leftSecondaryConfig = new TalonSRXConfiguration();
    TalonSRXConfiguration rightPrimaryConfig = new TalonSRXConfiguration();
    TalonSRXConfiguration rightSecondaryConfig = new TalonSRXConfiguration();

    // whatever configurations necessary like all .config methods

    // applying configs to motors

    leftPrimary.configAllSettings(leftPrimaryConfig);
    leftSecondary.configAllSettings(leftSecondaryConfig);
    rightPrimary.configAllSettings(rightPrimaryConfig);
    rightSecondary.configAllSettings(rightSecondaryConfig);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // inversion methods, carry over from 2020

  /**
   * Returns whether motors are inverted.
   * @return Boolean measure of motor inversion.
   */
  public boolean getInversion() {

    return inverted;

  }

  /**
   * Sets motor inversion state.
   * @param invert - Inverts motors.
   */
  public void setInversion(boolean invert) {

    inverted = invert;
    leftPrimary.setInverted(inverted);
    leftSecondary.setInverted(inverted);
    rightPrimary.setInverted(!inverted);
    rightSecondary.setInverted(!inverted);
    
  }

  /**
   * Toggles current inversion of motors.
   */
  public void toggleInversion() {

    setInversion(!inverted);

  }

  // speed methods

  /**
   * Returns the current maximum drive speed as a percentage of output.
   * @return Maximum drive speed.
   */
  public double getDriveSpeed() {

    return driveSpeed;

  }

  /**
   * Sets the maximum drive speed as a percentage of output.
   * @param driveSpeed - Maximum drive speed (percentage).
   */
  public void setDriveSpeed(double driveSpeed) {

    this.driveSpeed = driveSpeed;

  }

  /**
   * Sets speed of a motor at a percentage of output.
   * @param inputSpeed - Input speed (percentage).
   * @param useLeft - Whether to use the left-side motor (disregarding inversion).
   * @param controlled - Whether to control the input using driveSpeed.
   */
  public void setMotorSpeed(double inputSpeed, boolean useLeft, boolean controlled) {

    // if controlled, cap the input speed at driveSpeed (both percentages)

    double speed = controlled ? (driveSpeed * inputSpeed) : inputSpeed;

    // if ((not inverted and use right) or (inverted and use left)) set right speed
    // if ((inverted and use right) or (not inverted and use left)) set left speed
    
    if ((!inverted && !useLeft) || (inverted && useLeft)) {

      rightPrimary.set(ControlMode.PercentOutput, speed);

    } else {

      leftPrimary.set(ControlMode.PercentOutput, speed);
      
    }

  }

}
