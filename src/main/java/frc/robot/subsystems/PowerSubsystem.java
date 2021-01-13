/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerSubsystem extends SubsystemBase {

  // Singleton class:
  // private final static PowerSubsystem POWER_SUBSYSTEM = new PowerSubsystem();
  
  private final static int PDP_PORT = 5;
  private PowerDistributionPanel pdp;

  private final static double BROWNOUT_VOLTS = 9;
  private final static double BLACKOUT_VOLTS = 1;

  private double[] maxCurrentArray;

  /**
   * Creates a new PowerSubsystem.
   */
  public PowerSubsystem() {

    pdp = new PowerDistributionPanel(PDP_PORT);

    maxCurrentArray = new double[16];

  }

  // voltage methods (isBrowningOut * getVoltage from 2020)

  /**
   * Gets voltage of power distribution panel.
   * 
   * @return Current voltage of PDP.
   */
  public double getVoltage() {

    return pdp.getVoltage();

  }

  /**
   * Checks if system is browning out.
   * 
   * @return Whether system is browning out.
   */
  public boolean isBrowningOut() {

    return (pdp.getVoltage() <= BROWNOUT_VOLTS);

  }

  /**
   * Checks if system is blacking out.
   * 
   * @return Whether system is blacking out.
   */
  public boolean isBlackingOut() {

    return (pdp.getVoltage() <= BLACKOUT_VOLTS);

  }

  // current methods

  /**
   * Gets current from a certain channel
   * 
   * @param channel - Channel for motor/device.
   * @return Current.
   */
  public double getCurrent(int channel) {

    return pdp.getCurrent(channel);

  }

  /**
   * Sets maximum current for a certain channel.
   * 
   * @param channel    - Channel for motor/device.
   * @param maxCurrent - Maximum current.
   */
  public void setMaxCurrent(int channel, double maxCurrent) {

    maxCurrentArray[channel] = maxCurrent;

  }

  /**
   * Checks if current exceeds maximum current for certain channel.
   * @param channel - Channel for motor/device.
   * @return Whether current is greater than specified maximum current.
   */
  public boolean checkCurrent(int channel) {

    return (getCurrent(channel) > maxCurrentArray[channel]);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
