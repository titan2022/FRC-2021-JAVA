package frc.robot.config;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Control Panel logic definitions
 */
public class ControlPanelMap {
    public enum Button {
        // TODO: Add how many buttons there are
        elevatorUp(0), elevatorDown(1);// TODO: Find another way to do button configurations
                                       // Current method can have duplicated buttons numbers
    
    private final int value;
    Button(int value) { this.value = value;}
    public int getValue() { return value; }
    };

    private static final double JOYSTICK_DRIFT = 1e-2;

    private static Joystick leftPanels = OI.xinmotek1, rightPanels = OI.xinmotek2;
    
    // TODO: give names to methods
    public static double leftJoystick() 
    {
        double value = leftPanels.getX();
        return (Math.abs(value) < JOYSTICK_DRIFT) ? 0 : value;
    }

    public static double rightJoystick()
    {
        double value = rightPanels.getX();
        return (Math.abs(value) < JOYSTICK_DRIFT) ? 0 : value;
    }

    public static boolean elevatorUp()
    {
        return leftPanels.getRawButton(Button.elevatorUp.getValue());
    }
}
