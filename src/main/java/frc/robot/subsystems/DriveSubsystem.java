/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// using WPILib's docs' example from:
// https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html

public class DriveSubsystem extends SubsystemBase {

  // port numbers to be added later
  // add constants to file later

  private final static int LEFT_PRIMARY_PORT = 1;
  private final static int LEFT_SECONDARY_PORT = 2;
  private final static int RIGHT_PRIMARY_PORT = 3;
  private final static int RIGHT_SECONDARY_PORT = 4;
  private final static int PDP_PORT = 5;
  private final static double BROWNOUT_VOLTS = 9;
  private final static double BLACKOUT_VOLTS = 1;

  private TalonSRX leftPrimary, leftSecondary, rightPrimary, rightSecondary;
  private PowerDistributionPanel pdp;

  private boolean inverted;
  private double maxSpeed;

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

    pdp = new PowerDistributionPanel(PDP_PORT);

    inverted = false;
    maxSpeed = 1;
    
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
  public double getMaxSpeed() {

    return maxSpeed;

  }

  /**
   * Sets the maximum drive speed as a percentage of output.
   * @param maxSpeed - Maximum drive speed (percentage).
   */
  public void setmaxSpeed(double maxSpeed) {

    this.maxSpeed = maxSpeed;

  }

  /**
   * Sets speed of a motor at a percentage of output.
   * @param inputSpeed - Input speed (percentage).
   * @param useLeft - Whether to use the left-side motor (disregarding inversion).
   * @param controlled - Whether to control the input speed using maxSpeed.
   */
  public void setMotorSpeed(double inputSpeed, boolean useLeft, boolean controlled) {

    // if controlled, cap the input speed at maxSpeed (both percentages)

    double speed = controlled ? (maxSpeed * inputSpeed) : inputSpeed;

    // if ((not inverted and use right) or (inverted and use left)) set right speed
    // if ((inverted and use right) or (not inverted and use left)) set left speed
    
    if ((!inverted && !useLeft) || (inverted && useLeft)) {

      rightPrimary.set(ControlMode.PercentOutput, speed);

    } else {

      leftPrimary.set(ControlMode.PercentOutput, speed);
      
    }

  }

  /**
   * Sets motor speeds using tank drive
   * @param leftInputSpeed - Input speed for left-side motor (percentage).
   * @param rightInputSpeed - Input speed for right-side motor (percentage).
   * @param controlled - Whether to control the input speeds using maxSpeed.
   */
  public void tankDrive(double leftInputSpeed, double rightInputSpeed, boolean controlled) {

    setMotorSpeed(leftInputSpeed, true, controlled);
    setMotorSpeed(rightInputSpeed, false, controlled);

  }

  // braking methods (holdover from 2020)

  /**
   * Enables brake.
   */
  public void enableBrake() {

    leftPrimary.setNeutralMode(NeutralMode.Brake);
    rightPrimary.setNeutralMode(NeutralMode.Brake);

  }

  /**
   * Disables brake.
   */
  public void disableBrake() {

    leftPrimary.setNeutralMode(NeutralMode.Coast);
    rightPrimary.setNeutralMode(NeutralMode.Coast);

  }

  /**
   * Stops the robot.
   */
  public void stop() {

    leftPrimary.set(ControlMode.PercentOutput, 0);
    rightPrimary.set(ControlMode.PercentOutput, 0);

  }

  // voltage methods (isBrowningOut * getVoltage from 2020)

  /**
   * Gets voltage of power distribution panel.
   * @return Current voltage of PDP.
   */
  public double getVoltage() {

    return pdp.getVoltage();

  }

  /**
   * Checks if system is browning out.
   * @return Whether system is browning out.
   */
  public boolean isBrowningOut() {

    return (pdp.getVoltage() <= BROWNOUT_VOLTS);

  }

  /**
   * Checks if system is blacking out.
   * @return Whether system is blacking out.
   */
  public boolean isBlackingOut() {

    return (pdp.getVoltage() <= BLACKOUT_VOLTS);

  }

}
