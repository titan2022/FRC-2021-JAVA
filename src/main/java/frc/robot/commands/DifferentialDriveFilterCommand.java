// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motion.control.CustomKalmanFilter;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.sim.PhysicsSim;

public class DifferentialDriveFilterCommand extends CommandBase {

  private static final double STATE_STD_DEV = 0.1; // meters
  private static final double MEAS_STD_DEV = 0.01; // meters
  
  private final DifferentialDriveOdometryCommand odometryCommand;
  private final NavigationSubsystem navSub;
  private final CustomKalmanFilter filter; // vector: [xpos, xvel, xacc, ypos, yvel, yacc]
  private double prevT;

  /** Creates a new DifferentialDriveFilterCommand. */
  public DifferentialDriveFilterCommand(DifferentialDriveOdometryCommand odometryCommand, NavigationSubsystem navSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.odometryCommand = odometryCommand;
    this.navSub = navSub;

    filter = new CustomKalmanFilter(new SimpleMatrix(6, 1), SimpleMatrix.identity(6),
        SimpleMatrix.identity(6).scale(Math.pow(STATE_STD_DEV, 2)),
        SimpleMatrix.identity(6).scale(Math.pow(MEAS_STD_DEV, 2)), updateA(0),
        new SimpleMatrix(new double[][] { { 0, 0 }, { 1, 0 }, { 0, 0 }, { 0, 0 }, { 0, 1 }, { 0, 0 } }),
        SimpleMatrix.identity(6));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    prevT = PhysicsSim.getFPGATime();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    filter.setA(updateA(PhysicsSim.getFPGATime() - prevT));
    prevT = PhysicsSim.getFPGATime();

    // TODO: add filter.predictFilter(input(velocity)), bringing input from Xbox controller
    filter.updateFilter(odometryCommand.getOdometryVector());
    filter.updateFilter(navSub.getGyroVector());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Updates A matrix for specific time.
   * 
   * @param t - Time between filter runs.
   * @return Updated A matrix.
   */
  private SimpleMatrix updateA(double t) {

    return new SimpleMatrix(new double[][] { { 1, t, Math.pow(t, 2) / 2, 0, 0, 0 }, { 0, 1, t, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 1, t, Math.pow(t, 2) / 2 }, { 0, 0, 0, 0, 1, t }, { 0, 0, 0, 0, 0, 0 } });

  }

  public CustomKalmanFilter getFilter() {

    return filter;

  }

  /**
   * Returns the Kalman filter's current state.
   * 
   * @return Current Kalman filter state.
   */
  public SimpleMatrix getFilterState() {

    return filter.getState();

  }

  /**
   * Returns an element from the Kalman filter's current state.
   * 
   * @param row    - Row of state.
   * @param column - Row of state.
   * @return Element from current filter state.
   */
  public double getFilterStateElement(int row, int column) {

    return filter.getState().get(row, column);

  }

}
