package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Xbox button logic definitions
 */
public class XboxMap {
    private static final double JOYSTICK_DRIFT = 1e-2;
    private static final double RUMBLE_INTENSITY = 1; //(0,1]

    private static XboxController controller = Robot.oi.ps4;

    //Driving Controls 
	public static double left() {
		return controller.getY(Hand.kLeft);
    }
    
	public static double right() {
		return controller.getY(Hand.kRight);
    }

    public static boolean toggleBrakes() {
        return controller.getBumperPressed(Hand.kRight);
    }
    
    public static void startRumble(){
		controller.setRumble(RumbleType.kLeftRumble, RUMBLE_INTENSITY);
		controller.setRumble(RumbleType.kRightRumble, RUMBLE_INTENSITY);
    }
    
	public static void stopRumble(){
		controller.setRumble(RumbleType.kLeftRumble, 0);
		controller.setRumble(RumbleType.kRightRumble, 0);
	}

}
