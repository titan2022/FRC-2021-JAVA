// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import org.junit.jupiter.api.Test;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.kinematics.*;
import edu.wpi.first.wpiutil.math.Matrix;
import static org.junit.jupiter.api.Assertions.assertEquals;

/** Add your docs here. */
public class DifferentialDriveOdometryTest {

    @Test
    public void MovingInXDirectionOdometryTest()
    {
        DifferentialDriveOdometry x = new DifferentialDriveOdometry(1, 1, 7, 4, 0);
        Pose2d pose = x.getPosition(5, 1.0, 1.0);

        Pose2d truePose = new Pose2d(12, 4, new Rotation2d(0));

        assertEquals(pose, truePose);
    }

    @Test
    public void MovingInYDirectionOdometryTest()
    {
        DifferentialDriveOdometry x = new DifferentialDriveOdometry(1, 1, 7, 4, Math.PI/2);
        Pose2d pose = x.getPosition(5, 1.0, 1.0);

        Pose2d truePose = new Pose2d(7, 9, new Rotation2d(Math.PI/2));

        assertEquals(pose, truePose);
    }
}
