// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.PowerDistributionPanel;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// // import com.ctre.phoenix.motorcontrol.NeutralMode;
// // import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



// public class VHopperSubsystem extends SubsystemBase {

//   public static final int LEFT_PRIMARY_PORT = 1; //Motor 1: conveyor belt rotating
//   public static final int LEFT_SECONDARY_PORT = 2; //Motor 2: conveyor belt movement
//   public static final int RIGHT_PRIMARY_PORT = 3; //Motor 3: conveyor belt movement

  

//   Translation2d m_frontLeftLocation = new Translation2d(1, 1);
//   Translation2d m_frontRightLocation = new Translation2d(1, -1);


//   public static final WPI_TalonSRX leftPrimary = new WPI_TalonSRX(LEFT_PRIMARY_PORT)
//     , leftSecondary =new WPI_TalonSRX(LEFT_SECONDARY_PORT)
//     , rightPrimary = new WPI_TalonSRX(RIGHT_PRIMARY_PORT)
//     , rightSecondary = new WPI_TalonSRX(LEFT_PRIMARY_PORT);

  
//   private PowerDistributionPanel pdp;

  
//   private final static double BLACKOUT_VOLTS = 1; //
//   private final static double OVERCURRENT_AMPS = 100; //

//   private double[] maxCurrentArray;

//   /**
//    * Creates a new VHopperSubsystem.
//    */
//   public VHopperSubsystem() {

//     pdp = new PowerDistributionPanel(LEFT_PRIMARY_PORT);

//     maxCurrentArray = new double[16];

//   }

//   // voltage methods (isBrowningOut * getVoltage from 2020)

//   /**
//    * Gets voltage of power distribution panel.
//    * 
//    * @return Current voltage of PDP.
//    */
//   public double getVoltage() {

//     return pdp.getVoltage();

//   }

  
//   public void setLeftRotatorOutput (double rotatorOutput) {
      
//   }
  

//   /**
//    * Gets current from a certain channel
//    * 
//    * @param channel - Channel for motor/device.
//    * @return Current.
//    */
//   public double getCurrent(int channel) {

//     return pdp.getCurrent(channel);

//   }

//   /**
//    * Sets maximum current for a certain channel.
//    * 
//    * @param channel    - Channel for motor/device.
//    * @param maxCurrent - Maximum current.
//    */
//   public void setMaxCurrent(int channel, double maxCurrent) {

//     maxCurrentArray[channel] = maxCurrent;

//   }

//   /**
//    * Checks if current exceeds maximum current for certain channel.
//    * @param channel - Channel for motor/device.
//    * @return Whether current is greater than specified maximum current.
//    */
//   public PowerErrorCode checkChannelCurrent(int channel) {

//     if (getCurrent(channel) > maxCurrentArray[channel]) {

//       return PowerErrorCode.CHANNEL_OVERCURRENT;
      
//     } else {

//       return PowerErrorCode.NO_ERROR;

//     }

//   }


 
//   /**
//    * Checks if any channel's current exceeds its specified maximum current.
//    * @return Whether any channel's current is greater than its specified maximum current.
//    */
//   public PowerErrorCode checkAllChannelsCurrent() {

//     for (int i = 0; i < maxCurrentArray.length; i++) {

//       if (checkChannelCurrent(i) == PowerErrorCode.CHANNEL_OVERCURRENT) {

//         return PowerErrorCode.CHANNEL_OVERCURRENT;

//       }

//     }

//     return PowerErrorCode.NO_ERROR;

//   }

//   public void setRotationBrakes () {

//   }

//   /**
//    * Gets the system's total current.
//    * @return Total current.
//    */
//   public double getSystemCurrent() {

//     return pdp.getTotalCurrent();

//   }

  

//   /**
//    * Checks if any PowerErrorCodes will be thrown by the power subsystem.
//    * @return Most pressing PowerErrorCode from subsystem.
//    */
  
    

  

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

// }
