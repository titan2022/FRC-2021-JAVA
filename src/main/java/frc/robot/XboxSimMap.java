package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class XboxSimMap extends XboxMap {

    private static final double SIM_JOYSTICK_DRIFT = 0.01;

    private static XboxController simController = new XboxController(0);

    public static double left() {
      double value = simController.getY(Hand.kLeft);
      return (Math.abs(value) < SIM_JOYSTICK_DRIFT) ? 0 : value;
    }

    public static double right() {
      double value = simController.getY(Hand.kRight);
      return (Math.abs(value) < SIM_JOYSTICK_DRIFT) ? 0 : value;
    }

}