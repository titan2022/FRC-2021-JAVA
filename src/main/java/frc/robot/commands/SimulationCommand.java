/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
//package frc.robot.motionGeneration.rmpFlow;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpiutil.math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command that uses an example subsystem, like a boss.
 */
public class SimulationCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //MatBuilder DifferentialJacobianBuilder;
  private Matrix DifferentialJacobian;
  private double previousTime, currentTime, deltaT;
  private double vl, vr, newPosX, newPosY;
  private Field2d f;
  private Timer timer;
  private DifferentialDriveKinematics object;
  private Rotation2d newRotation;
  

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
    timer.start();
    previousTime = timer.get();

    f = new Field2d();
    f.setRobotPose(7, 3, new Rotation2d(0));

    vl = 5;
    vr = 10;

    SmartDashboard.putNumber("left", 5);
    SmartDashboard.putNumber("right", 10);

    object = new DifferentialDriveKinematics(1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //import my kinematics class and run the kinematics command on the previous position and the given left and right wheel velocities
    currentTime = timer.get();
    deltaT = currentTime - previousTime;

    vl = SmartDashboard.getNumber("left", 5);
    vr = SmartDashboard.getNumber("right", 10);

    //use left and right velocity and phi
    DifferentialJacobian = object.getAbsoluteVelocity(vl, vr, f.getRobotPose().getRotation().getRadians());

    //Set new rotation, x position, and y positions using our differential Jacobian.    
    newRotation = new Rotation2d(DifferentialJacobian.get(2,0) * deltaT + f.getRobotPose().getRotation().getRadians());
    newPosX = DifferentialJacobian.get(0,0) * deltaT + f.getRobotPose().getTranslation().getX();
    newPosY = DifferentialJacobian.get(1,0) * deltaT + f.getRobotPose().getTranslation().getY();

    //add intgrated values to the current position
    f.setRobotPose(newPosX, newPosY, newRotation);
    
    previousTime = currentTime;

    //If the robot goes out of bounds, place the robot back on the field. 
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
