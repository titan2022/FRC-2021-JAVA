package frc.robot.kinematics;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

/** Add your docs here. */
public class DifferentialDriveKinematicsTest {

    @Test
    public void TestGetAbsoluteVelocity()
    {
        DifferentialDriveKinematics x = new DifferentialDriveKinematics(1, 1);
        double testXVelocity = x.getAbsoluteVelocity(1, 1, 0).get(0, 0);
        double testYVelocity = x.getAbsoluteVelocity(1, 1, 0).get(1, 0);
        double testThetaVelocity = x.getAbsoluteVelocity(1, 1, 0).get(2, 0);

        double trueXVelocity = 1;
        double trueYVelocity = 0;
        double trueThetaVelocity = 0;

        assertEquals(trueXVelocity, testXVelocity);
        assertEquals(trueYVelocity, testYVelocity);
        assertEquals(trueThetaVelocity, testThetaVelocity);
    }

}
