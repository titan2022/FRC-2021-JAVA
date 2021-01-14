package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class OI {
    public enum JoystickButtons { a, b1 }; //TODO: Add how many buttons there are

    public XboxController ps4;//, xbox;
    private static final int PS4_PORT = 1;

    public Joystick xinmotek1, xinmotek2;
    private static final int XINMOTEK1_PORT = 2, XINMOTEK2_PORT = 3;

    /**
     * Holds access to all human interface hardware
     */
    public OI()
    {
        ps4 = new XboxController(PS4_PORT);
        xinmotek1 = new Joystick(XINMOTEK1_PORT);
        xinmotek2 = new Joystick(XINMOTEK2_PORT);
    }
}
