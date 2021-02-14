// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motion.generation.rmpflow.CollisionAvoidance;
import frc.robot.motion.generation.rmpflow.GoalAttractor;
import frc.robot.motion.generation.rmpflow.RMPRoot;

public class MotionGenerationSubsystem extends SubsystemBase {

  private RMPRoot root;
  private ArrayList<CollisionAvoidance> collisionAvoiders;
  private ArrayList<GoalAttractor> goalAttractors;
  // private SimpleMatrix x;
  // private SimpleMatrix x_dot;

  /** Creates a new MotionGenerationSubsystem. */
  public MotionGenerationSubsystem() {

    collisionAvoiders = new ArrayList<CollisionAvoidance>();
    goalAttractors = new ArrayList<GoalAttractor>();
    root = new RMPRoot("root");
    // x = new SimpleMatrix(1, 2);
    // x_dot = new SimpleMatrix(1, 2);
    
  }

  /**
   * Creates a new MotionGenerationSubsystem.
   * @param collisionAvoiders - ArrayList of RMP CollisionAvoidance objects.
   * @param goalAttractors - ArrayList of RMP GoalAttractor objects.
   * @param root - RMP Root object.
   */
  public MotionGenerationSubsystem(ArrayList<CollisionAvoidance> collisionAvoiders, ArrayList<GoalAttractor> goalAttractors, RMPRoot root) {

    setCollisionAvoiders(collisionAvoiders);
    setGoalAttractors(goalAttractors);
    this.root = root;
    // setX(x);
    // setXDot(x_dot);

  }

  /**
   * Sets new ArrayList of RMP CollisionAvoidance objects. Replaces previous CollisionAvoidances.
   * @param collisionAvoiders - ArrayList of RMP CollisionAvoidance objects.
   */
  public void setCollisionAvoiders(ArrayList<CollisionAvoidance> collisionAvoiders) {

    this.collisionAvoiders = collisionAvoiders;

  }

  /**
   * Sets new ArrayList of RMP GoalAttractor objects. Replaces previous GoalAttractors.
   * @param collisionAvoiders - ArrayList of RMP GoalAttractor objects.
   */
  public void setGoalAttractors(ArrayList<GoalAttractor> goalAttractors) {

    this.goalAttractors = goalAttractors;

  }

  /**
   * Gets ArrayList of RMP CollisionAvoidance objects.
   * @return ArrayList of RMP CollisionAvoidance objects.
   */
  public ArrayList<CollisionAvoidance> getCollisionAvoiders() {

    return collisionAvoiders;

  }

  /**
   * Gets ArrayList of RMP GoalAttractor objects.
   * @return ArrayList of RMP GoalAttractor objects.
   */
  public ArrayList<GoalAttractor> getGoalAttractors() {

    return goalAttractors;

  }

  /**
   * Adds to ArrayList of RMP GoalAttactor objects.
   * @param goalAttractor
   */
  public void addGoalAttractor(GoalAttractor goalAttractor) {

    goalAttractors.add(goalAttractor);

  }

  public void addCollisionAvoider(CollisionAvoidance collisionAvoider) {

    collisionAvoiders.add(collisionAvoider);
  
  }

  public CollisionAvoidance removeCollisionAvoider(int index) {

    return collisionAvoiders.remove(index);

  }

  public GoalAttractor removeGoalAttractor(int index) {

    return goalAttractors.remove(index);

  }

  // public SimpleMatrix getX() {

  //   return x;

  // }

  // public SimpleMatrix getXDot() {

  //   return x_dot;

  // }

  // public void setX(SimpleMatrix x) {

  //   this.x = x;

  // }

  // public void setXDot(SimpleMatrix x_dot) {

  //   this.x_dot = x_dot;

  // }

  public SimpleMatrix rootSolve(SimpleMatrix x, SimpleMatrix x_dot) {

    return root.solve(x, x_dot);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
