// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motion.generation.rmpflow.CollisionAvoidance;
import frc.robot.motion.generation.rmpflow.GoalAttractor;
import frc.robot.motion.generation.rmpflow.RMPRoot;

public class GenerationSubsystem extends SubsystemBase {

  private RMPRoot root = new RMPRoot("root");
  private ArrayList<CollisionAvoidance> collisionAvoiders;
  private ArrayList<GoalAttractor> goalAttractors;

  /** Creates a new GenerationSubsystem. */
  public GenerationSubsystem() {

    collisionAvoiders = new ArrayList<CollisionAvoidance>();
    goalAttractors = new ArrayList<GoalAttractor>();
    
  }

  public GenerationSubsystem(ArrayList<CollisionAvoidance> collisionAvoiders, ArrayList<GoalAttractor> goalAttractors) {

    setCollisionAvoiders(collisionAvoiders);
    setGoalAttractors(goalAttractors);

  }

  public void setCollisionAvoiders(ArrayList<CollisionAvoidance> collisionAvoiders) {

    this.collisionAvoiders = collisionAvoiders;

  }

  public void setGoalAttractors(ArrayList<GoalAttractor> goalAttractors) {

    this.goalAttractors = goalAttractors;

  }

  public void addGoalAttractor(GoalAttractor goalAttractor) {

    goalAttractors.add(goalAttractor);

  }

  public void addCollisionAvoider(CollisionAvoidance collisionAvoider) {

    collisionAvoiders.add(collisionAvoider);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
