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

  private final RMPRoot root;
  private ArrayList<CollisionAvoidance> collisionAvoiders;
  private ArrayList<GoalAttractor> goalAttractors;
  
  /** Creates a new MotionGenerationSubsystem. */
  public MotionGenerationSubsystem() {

    collisionAvoiders = new ArrayList<CollisionAvoidance>();
    goalAttractors = new ArrayList<GoalAttractor>();
    root = new RMPRoot("root");
    
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
   * @param goalAttractors - ArrayList of RMP GoalAttractor objects.
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
   * @param goalAttractor - RMP GoalAttractor object.
   */
  public void addGoalAttractor(GoalAttractor goalAttractor) {

    goalAttractors.add(goalAttractor);

  }

  /**
   * Adds to ArrayList of RMP CollisionAvoidance objects.
   * @param collisionAvoider - RMP CollisionAvoidance object.
   */
  public void addCollisionAvoider(CollisionAvoidance collisionAvoider) {

    collisionAvoiders.add(collisionAvoider);
  
  }

  /**
   * Removes RMP CollisionAvoidance object from ArrayList with specified index.
   * @param index - Index of RMP CollisionAvoidance object.
   * @return Removed RMP CollisionAvoidance object.
   */
  public CollisionAvoidance removeCollisionAvoider(int index) {

    return collisionAvoiders.remove(index);

  }

  /**
   * Removes RMP GoalAttractor object from ArrayList with specified index.
   * @param index - Index of RMP GoalAttractor object.
   * @return Removed RMP GoalAttractor object.
   */
  public GoalAttractor removeGoalAttractor(int index) {

    return goalAttractors.remove(index);

  }

  /**
   * Removes specified RMP CollisionAvoidance object from ArrayList.
   * @param collisionAvoider - RMP CollisionAvoidance object to be removed.
   * @return If RMP CollisionAvoidance object was removed (boolean).
   */
  public boolean removeCollisionAvoider(CollisionAvoidance collisionAvoider) {

    return collisionAvoiders.remove(collisionAvoider);

  }

  /**
   * Removes specified RMP GoalAttractor object from ArrayList.
   * @param goalAttractor - RMP GoalAttractor object to be removed.
   * @return If RMP GoalAttractor object was removed (boolean).
   */
  public boolean removeGoalAttractor(GoalAttractor goalAttractor) {

    return goalAttractors.remove(goalAttractor);

  }

  /**
   * Gets RMP Root object.
   * @return RMP Root object.
   */
  public RMPRoot getRoot() {

    return root;

  }

  public SimpleMatrix rootSolve(SimpleMatrix x, SimpleMatrix x_dot) {

    return root.solve(x, x_dot);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
