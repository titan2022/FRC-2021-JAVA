/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
//package frc.robot.motionGeneration.rmpFlow;

import frc.wpilibjTemp.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.kinematics.DifferentialDriveKinematics;

import javax.swing.JFrame;
import edu.wpi.first.wpiutil.math.*;
import java.lang.Math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.wpilibjTemp.Field2d;

/**
 * An example command that uses an example subsystem, like a boss.
 */
public class SimulationCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //MatBuilder DifferentialJacobianBuilder;
  private Matrix DifferentialJacobian;
  private double vl;
  private double vr;
  private Field2d f;
  private Timer timer;
  private double previousTime;
  private DifferentialDriveKinematics object;
  private Rotation2d rotationamount;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SimulationCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //start the timer
    //set the speeds of each wheel and stuff
    //set up the simulation environment w position

    timer = new Timer();
    f = new Field2d();
    timer.start();
    previousTime = timer.get();
    f.setRobotPose(7, 3, new Rotation2d(0));
    vl = 5;
    vr = 10;

    object = new DifferentialDriveKinematics(1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //import my kinematics class and run the kinematics command on the previous position and the given left and right wheel velocities
    double currentTime = timer.get();
    double deltat = 0.5 * (currentTime - previousTime);
    //use left and right velocity and phi
    DifferentialJacobian = object.getAbsoluteVelocity(vl, vr, f.getRobotPose().getRotation().getRadians());

    rotationamount = new Rotation2d(DifferentialJacobian.get(2,0) * deltat + f.getRobotPose().getRotation().getRadians());

    //add intgrated values to the current position
    f.setRobotPose(DifferentialJacobian.get(0,0) * deltat + f.getRobotPose().getTranslation().getX(), DifferentialJacobian.get(1,0) * deltat + f.getRobotPose().getTranslation().getY(), rotationamount);
    previousTime = currentTime;
    
    if(f.getRobotPose().getTranslation().getX() <= 0 || f.getRobotPose().getTranslation().getX() >= 15.98 || f.getRobotPose().getTranslation().getY() <= 0 || f.getRobotPose().getTranslation().getY() >= 8.21)
    {
      f.setRobotPose(7, 3, new Rotation2d(0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
