// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutoNavChallenge;
import frc.robot.commands.DifferentialDriveFilterCommand;
import frc.robot.commands.DifferentialDriveOdometryCommand;
import frc.robot.commands.FieldDisplayCommand;
import frc.robot.commands.RMPDrive;
import frc.robot.motion.generation.rmpflow.RMPRoot;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

/** Add your docs here. */
public class AutonavContainer implements RobotContainer
{
    private final ParallelCommandGroup autoGroup;
    private final ParallelCommandGroup teleopGroup;
    private final DifferentialDriveSubsystem diffDriveSub;
    private final NavigationSubsystem navSub;

    public AutonavContainer(Translation2d... waypoints) {
        diffDriveSub = new DifferentialDriveSubsystem(getLeftDiffDriveTalonConfig(), getRightDiffDriveTalonConfig(), true);
        navSub = new NavigationSubsystem(diffDriveSub);
        DifferentialDriveOdometryCommand odometryCommand = new DifferentialDriveOdometryCommand(diffDriveSub, navSub);
        DifferentialDriveFilterCommand diffDriveFilter = new DifferentialDriveFilterCommand(odometryCommand, navSub);
        RMPRoot root = new RMPRoot("root");
        RMPDrive rmpDrive = new RMPDrive(root, diffDriveSub, diffDriveFilter);
        FieldDisplayCommand fieldDisplayCommand = new FieldDisplayCommand("Auto Nav Challenge");
        AutoNavChallenge autoNav = new AutoNavChallenge(rmpDrive, waypoints);

        autoGroup = new ParallelCommandGroup(fieldDisplayCommand, autoNav, odometryCommand, diffDriveFilter);
        teleopGroup = new ParallelCommandGroup();
    }

    @Override
    public Command getAutonomousCommand() {
        return autoGroup;
    }

    @Override
    public Command getTeleopCommand() {
        return teleopGroup;
    }

        /**
     * Method containing the talon configuration of the left side of the differential drive.
     * @return The talon configuration of the left side of the drive.
     */
    public TalonSRXConfiguration getLeftDiffDriveTalonConfig()
    {
        TalonSRXConfiguration talon = new TalonSRXConfiguration();
        // Add configs here:

        return talon;
    }

    /**
     * Method containing the talon configuration of the right side of the differential drive.
     * @return The talon configuration of the right side of the drive.
     */
    public TalonSRXConfiguration getRightDiffDriveTalonConfig()
    {
        TalonSRXConfiguration talon = new TalonSRXConfiguration();
        // Add configs here:

        return talon;
    }

}
