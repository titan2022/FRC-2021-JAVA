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
  
  private final static int PDP_PORT = 5;
  private PowerDistributionPanel pdp;

  private final static double BROWNOUT_VOLTS = 9;
  private final static double BLACKOUT_VOLTS = 1;

  /**
   * Creates a new PowerSubsystem.
   */
  public PowerSubsystem() {

    pdp = new PowerDistributionPanel(PDP_PORT);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
