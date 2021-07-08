package frc.robot.config;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Hardware for human interface
 */
public final class OI {
    // Controller OI
    private static final int PS4_PORT = 1, XBOX_PORT = 0; // Drive station ports will be different from simulator ports
    public static final XboxController ps4 = new XboxController(PS4_PORT), xbox = new XboxController(XBOX_PORT);

    // Control Panel OI
    private static final int XINMOTEK1_PORT = 2, XINMOTEK2_PORT = 3;
    public static final Joystick xinmotek1 = new Joystick(XINMOTEK1_PORT), xinmotek2 = new Joystick(XINMOTEK2_PORT);
}