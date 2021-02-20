package frc.robot.subsystems;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public interface DriveSubsystem {
    void setVelocities(ChassisSpeeds velocities);
}
