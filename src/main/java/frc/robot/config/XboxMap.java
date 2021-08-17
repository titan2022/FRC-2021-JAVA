package frc.robot.config;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Xbox button logic definitions
 */
public class XboxMap {
  private static final double JOYSTICK_DRIFT = .09;
  private static final double MAX_RUMBLE_INTENSITY = 1; // [0,1]
  
  // Orientation Area of Effects
  private static final double ORIENTATION_LOWER_RADIUS = .5;

  // Main Driver and Auxilliary Driver
  private static final XboxController controller = OI.ps4;
  private static final XboxController auxController = OI.xbox;

  // Differential Driving Controls
  public static double leftWheel() { // TODO: fix this confusing mess of a controller configuration
    double value = -controller.getY(Hand.kLeft);
    return applyDeadband(value, JOYSTICK_DRIFT);
  }

  public static double rightWheel() {
    double value = -controller.getY(Hand.kRight);
    return applyDeadband(value, JOYSTICK_DRIFT);
  }

  // Swerve Driving Controls
  public static double translationX() {
    double value = controller.getX(Hand.kLeft);
    return applyDeadband(value, JOYSTICK_DRIFT);
  }

  public static double translationY() { // TODO: fix this confusing mess of a controller configuration
    double value = -controller.getY(Hand.kLeft);
    return applyDeadband(value, JOYSTICK_DRIFT);
  }

  public static double orientationX() {
    double y = -controller.getY(Hand.kRight);
    double x = controller.getX(Hand.kRight);
    if(x*x + y*y < ORIENTATION_LOWER_RADIUS*ORIENTATION_LOWER_RADIUS)
      return 0;
    return x;
  }

  public static double orientationY() {
    double y = -controller.getY(Hand.kRight);
    double x = controller.getX(Hand.kRight);
    if(x*x + y*y < ORIENTATION_LOWER_RADIUS*ORIENTATION_LOWER_RADIUS)
      return 0;
    return y;
  }

  public static boolean toggleBrakes() {
    return controller.getBumperPressed(Hand.kRight);
  }

  // V-Hopper Controls

  public static double hopperPct() {
    return controller.getTriggerAxis(Hand.kLeft);
  }

  public static double scissorSpeed() {
    return controller.getTriggerAxis(Hand.kRight);
  }

  // Intake Controls

  public static boolean toggleWrist() {
    return controller.getBumperPressed(Hand.kLeft);
  }
  

  // General controls

  /**
   * Sets controller rumble
   * @param intensity [0,1]
   */
  public static void setRumble(double intensity)
  {
    controller.setRumble(RumbleType.kLeftRumble, intensity);
		controller.setRumble(RumbleType.kRightRumble, intensity);
  }

  public static void startRumble() {
	  controller.setRumble(RumbleType.kLeftRumble, MAX_RUMBLE_INTENSITY);
		controller.setRumble(RumbleType.kRightRumble, MAX_RUMBLE_INTENSITY);
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