/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// using WPILib's docs' example from:
// https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html

public class DriveSubsystem extends SubsystemBase {

  // port numbers to be added later

  private final static int LEFT_PRIMARY_PORT = 1;
  private final static int LEFT_SECONDARY_PORT = 2;
  private final static int RIGHT_PRIMARY_PORT = 3;
  private final static int RIGHT_SECONDARY_PORT = 4;

  private TalonSRX leftPrimary, leftSecondary, rightPrimary, rightSecondary;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    leftPrimary = new TalonSRX(LEFT_PRIMARY_PORT);
    leftSecondary = new TalonSRX(LEFT_SECONDARY_PORT);
    rightPrimary = new TalonSRX(RIGHT_PRIMARY_PORT);
    rightSecondary = new TalonSRX(RIGHT_SECONDARY_PORT);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
