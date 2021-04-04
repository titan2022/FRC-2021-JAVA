/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.motion.generation.rmpflow.demos;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.motion.generation.rmpflow.*;

public class RMPDemoCommand extends CommandBase {
  private Timer timer;
  private double previousTime;
  private Field2d field;
  private RMPRoot root;
  private ArrayList<CollisionAvoidance> collisionAvoiders = new ArrayList<CollisionAvoidance>();
  private ArrayList<GoalAttractor> goalAttractors = new ArrayList<GoalAttractor>();
  private SimpleMatrix x_ddot, x_dot, x;

  /**
   * Creates a new RMPDemoCommand.
   */
  public RMPDemoCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    field = new Field2d();
    timer.start();
    previousTime = timer.get();
    System.out.println("The first");
    root = new RMPRoot("root");

    double alpha = 1e-5, eta = 2, epsilon = .2;

    collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test1", root, new SimpleMatrix(1, 2,  false, new double[] {6.752, 4.35}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test2", root, new SimpleMatrix(1, 2, false, new double[] {4.467, 6.415}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test3", root, new SimpleMatrix(1, 2, false, new double[] {4.4966, 1.83}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test4", root, new SimpleMatrix(1, 2, false, new double[] {9.037, 1.7576}), 1, epsilon, alpha, eta));
    collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test5", root, new SimpleMatrix(1, 2, false, new double[] {8.964, 6.44729}), 1, epsilon, alpha, eta));
    
    goalAttractors.add(new GoalAttractor("Goal Attractor Test", root, new SimpleMatrix(1, 2, false, new double[] {12.728, 4.306}), 10, 1, 10, 1, 2, 2, .005));

    //Initial setup
    x = new SimpleMatrix(1, 2, false, new double[] {1.6, 4.3});
		x_dot = new SimpleMatrix(1, 2, false, new double[] {0, 0});
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Rotation2d theta = new Rotation2d(field.getRobotPose().getTranslation().getX() - x.get(0), field.getRobotPose().getTranslation().getY() - x.get(1));

    x = new SimpleMatrix(1, 2, false, new double[] {field.getRobotPose().getTranslation().getX(), field.getRobotPose().getTranslation().getY()});

    x_ddot = root.solve(x, x_dot);
    double[] newState = solveIntegration(.01*5, x_ddot, x_dot, x);
    //double[] newState = solveIntegration(timer.get() - previousTime, x_ddot, x_dot, x);
		x_dot.set(1, newState[3]);
		x_dot.set(0, newState[2]);
		//x.set(1, newState[1]);
    //x.set(0, newState[0]);
    //field.setRobotPose(x.get(0), x.get(1), theta);
    field.setRobotPose(newState[0], newState[1], theta);

    //Update data
    SmartDashboard.putNumber("Time (s)", previousTime);
    SmartDashboard.putNumber("X Pos", field.getRobotPose().getTranslation().getX());
    SmartDashboard.putNumber("Y Pos", field.getRobotPose().getTranslation().getY());
    SmartDashboard.putNumber("theta Pos", field.getRobotPose().getRotation().getDegrees());
    SmartDashboard.putData("Field", field);
    previousTime = timer.get();
  }

  //Dynamics of system
	public static double[] solveIntegration(double deltaT, SimpleMatrix x_ddot, SimpleMatrix x_dot, SimpleMatrix x)
	{
		double[] state = new double[4];
		state[3] = x_dot.get(1) + x_ddot.get(1) * deltaT;//y value
		state[2] = x_dot.get(0) + x_ddot.get(0) * deltaT;//x value
		state[1] = x.get(1) + .5 * (x_dot.get(1) + state[3]) * deltaT;//Trapezoid integration
		state[0] = x.get(0) + .5 * (x_dot.get(0) + state[2]) * deltaT;
		return state;
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
