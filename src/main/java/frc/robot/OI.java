package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Hardware for human interface
 */
public final class OI {
    // Controller OI
    public final XboxController ps4 = new XboxController(PS4_PORT), xbox = new XboxController(XBOX_PORT);;
    private static final int PS4_PORT = 0, XBOX_PORT = 1; // Drive station ports will be different from simulator ports

    // Control Panel OI
    public final Joystick xinmotek1 = new Joystick(XINMOTEK1_PORT), xinmotek2 = new Joystick(XINMOTEK2_PORT);
    private static final int XINMOTEK1_PORT = 2, XINMOTEK2_PORT = 3;
}
