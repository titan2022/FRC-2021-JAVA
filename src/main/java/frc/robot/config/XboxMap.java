package frc.robot.config;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Xbox button logic definitions
 */
public class XboxMap {
  private static final double JOYSTICK_DRIFT = .09;
  private static final double RUMBLE_INTENSITY = 1; // [0,1]

  // Main Driver and Auxilliary Driver
  private static final XboxController controller = OI.ps4;
  private static final XboxController auxController = OI.xbox;

  // Driving Controls 
	public static double left() {
    double value = -controller.getY(Hand.kLeft);
    return applyDeadband(value, JOYSTICK_DRIFT);
  }

  public static double leftX() {
    double value = controller.getX(Hand.kLeft);
    return applyDeadband(value, JOYSTICK_DRIFT);
  }

  public static double leftY() {
    return left();
  }

    
	public static double right() {
    double value = -controller.getY(Hand.kRight);
    return applyDeadband(value, JOYSTICK_DRIFT);
  }

  public static double rightX() {
    double value = controller.getX(Hand.kRight);
    return applyDeadband(value, JOYSTICK_DRIFT);
  }

  public static double rightY() {
    return right();
  }

  public static boolean toggleBrakes() {
    return controller.getBumperPressed(Hand.kRight);
  }
    
  public static void startRumble() {
	  controller.setRumble(RumbleType.kLeftRumble, RUMBLE_INTENSITY);
		controller.setRumble(RumbleType.kRightRumble, RUMBLE_INTENSITY);
  }
    
	public static void stopRumble() {
		controller.setRumble(RumbleType.kLeftRumble, 0);
		controller.setRumble(RumbleType.kRightRumble, 0);
  }
  
  /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value value to clip
   * @param deadband range around zero
   */
  public static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}