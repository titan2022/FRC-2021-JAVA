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
  private final PowerDistributionPanel pdp;

  private final static double BROWNOUT_VOLTS = 9;
  private final static double BLACKOUT_VOLTS = 1;
  private final static double OVERCURRENT_AMPS = 100;

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
  public PowerErrorCode isBrowningOut() {

    if (pdp.getVoltage() <= BROWNOUT_VOLTS) {

      return PowerErrorCode.BROWNOUT;

    } else {

      return PowerErrorCode.NO_ERROR;
      
    }

  }

  /**
   * Checks if system is blacking out.
   * 
   * @return Whether system is blacking out.
   */
  public PowerErrorCode isBlackingOut() {

    if (pdp.getVoltage() <= BLACKOUT_VOLTS) {

      return PowerErrorCode.BLACKOUT;

    } else {

      return PowerErrorCode.NO_ERROR;
      
    }

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
  public PowerErrorCode checkChannelCurrent(int channel) {

    if (getCurrent(channel) > maxCurrentArray[channel]) {

      return PowerErrorCode.CHANNEL_OVERCURRENT;
      
    } else {

      return PowerErrorCode.NO_ERROR;

    }

  }

  /**
   * Checks if any channel's current exceeds its specified maximum current.
   * @return Whether any channel's current is greater than its specified maximum current.
   */
  public PowerErrorCode checkAllChannelsCurrent() {

    for (int i = 0; i < maxCurrentArray.length; i++) {

      if (checkChannelCurrent(i) == PowerErrorCode.CHANNEL_OVERCURRENT) {

        return PowerErrorCode.CHANNEL_OVERCURRENT;

      }

    }

    return PowerErrorCode.NO_ERROR;

  }

  /**
   * Gets the system's total current.
   * @return Total current.
   */
  public double getSystemCurrent() {

    return pdp.getTotalCurrent();

  }

  /**
   * Checks if system's current exceeds its specified maximum current.
   * @return Whether system's current is greater than its specified maximum current.
   */
  public PowerErrorCode checkSystemCurrent() {

    if (getSystemCurrent() > OVERCURRENT_AMPS) {

      return PowerErrorCode.SYSTEM_OVERCURRENT;
      
    } else {

      return PowerErrorCode.NO_ERROR;

    }

  }

  /**
   * Checks if any PowerErrorCodes will be thrown by the power subsystem.
   * @return Most pressing PowerErrorCode from subsystem.
   */
  public PowerErrorCode checkStatus() {

    if (isBlackingOut() == PowerErrorCode.BLACKOUT) {

      return PowerErrorCode.BLACKOUT;

    } else if (isBrowningOut() == PowerErrorCode.BROWNOUT) {

      return PowerErrorCode.BROWNOUT;

    } else if (checkSystemCurrent() == PowerErrorCode.SYSTEM_OVERCURRENT) {

      return PowerErrorCode.SYSTEM_OVERCURRENT;

    } else if (checkAllChannelsCurrent() == PowerErrorCode.CHANNEL_OVERCURRENT) {

      return PowerErrorCode.CHANNEL_OVERCURRENT;

    } else {

      return PowerErrorCode.NO_ERROR;

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
