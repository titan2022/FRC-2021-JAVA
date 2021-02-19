package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motion.control.CustomKalmanFilter;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.sim.PhysicsSim;

/**
 * 
 */
public class DifferentialDriveFilterCommand extends CommandBase {
  private static final double STATE_STD_DEV = 0.1; // meters
  private static final double MEAS_STD_DEV = 0.01; // meters
  
  private final CustomKalmanFilter filter; // vector: [xpos, xvel, xacc, ypos, yvel, yacc]
  private final DifferentialDriveOdometryCommand odometryCommand;
  private final NavigationSubsystem navSub;
  private double prevT;

  /**
   * Creates a new DifferentialDriveFilterCommand with odometry and navigation.
   * @param odometryCommand - Odometry for robot.
   * @param navSub - Navigation subsystem.
   */
  public DifferentialDriveFilterCommand(DifferentialDriveOdometryCommand odometryCommand, NavigationSubsystem navSub) {
    filter = new CustomKalmanFilter(new SimpleMatrix(6, 1), SimpleMatrix.identity(6),
        SimpleMatrix.identity(6).scale(Math.pow(STATE_STD_DEV, 2)),
        SimpleMatrix.identity(6).scale(Math.pow(MEAS_STD_DEV, 2)), updateA(0),
        new SimpleMatrix(new double[][] { { 0, 0 }, { 1, 0 }, { 0, 0 }, { 0, 0 }, { 0, 1 }, { 0, 0 } }),
        SimpleMatrix.identity(6));
    this.odometryCommand = odometryCommand;
    this.navSub = navSub;
  }

  @Override
  public void initialize() {
    prevT = PhysicsSim.getFPGATime();
  }

  @Override
  public void execute() {
    filter.setA(updateA(PhysicsSim.getFPGATime() - prevT));
    prevT = PhysicsSim.getFPGATime();

    // TODO: add filter.predictFilter(input(velocity)), bringing input from Xbox controller

    filter.updateFilter(odometryCommand.getOdometryVector());
    filter.updateFilter(navSub.getGyroVector());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Updates A matrix for specific time.
   * 
   * @param dt - Time between filter runs.
   * @return Updated A matrix.
   */
  private SimpleMatrix updateA(double dt) {
    return new SimpleMatrix(new double[][] { { 1, dt, Math.pow(dt, 2) / 2, 0, 0, 0 }, { 0, 1, dt, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 1, dt, Math.pow(dt, 2) / 2 }, { 0, 0, 0, 0, 1, dt }, { 0, 0, 0, 0, 0, 0 } });
  }

  /**
   * Gets a copy of the Kalman filter.
   * @return Kalman filter.
   */
  public CustomKalmanFilter getFilterCopy() {
    return new CustomKalmanFilter(filter);
  }

  /**
   * Returns the Kalman filter's current state.
   * @return Current Kalman filter state.
   */
  public SimpleMatrix getFilterState() {
    return filter.getState();
  }

  /**
   * Returns the Kalman filter's estimated robot pose.
   * @return Current filtered pose.
   */
  public Pose2d getFilteredPose() {
    return new Pose2d(getFilterStateElement(0, 0), getFilterStateElement(0, 3), new Rotation2d()); // TODO: add theta to filter
  }

  /**
   * Returns an element from the Kalman filter's current state.
   * @param row    - Row of state.
   * @param column - Row of state.
   * @return Element from current filter state.
   */
  private double getFilterStateElement(int row, int column) {
    return getFilterState().get(row, column);
  }
}
