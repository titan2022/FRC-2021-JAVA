package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.subsystems.sim.PhysicsSim;

public class SwerveDriveSubsystem extends SubsystemBase
{
    // Physical parameters
  public static final double ROBOT_TRACK_WIDTH = 26.75/39.37; // meter
  public static final double WHEEL_RADIUS = 6/39.37; // meters
  public static final double ENCODER_TICKS = 4096; // Ticks/rotation of CTREMagEncoder
  public static final double METERS_PER_TICK = WHEEL_RADIUS * 2 * Math.PI / ENCODER_TICKS;
  public static final double GEARING_REDUCTION = 7.29; // TODO: Get the correct gearing ratio
    
  // Port numbers to be added later
  private static final int LEFT_PRIMARY_PORT = 1;
  private static final int LEFT_SECONDARY_PORT = 2;
  private static final int RIGHT_PRIMARY_PORT = 3;
  private static final int RIGHT_SECONDARY_PORT = 4;
  private static final int LEFT_PRIMARY_ROTATOR_PORT = 5;
  private static final int LEFT_SECONDARY_ROTATOR_PORT = 6;
  private static final int RIGHT_PRIMARY_ROTATOR_PORT = 7;
  private static final int RIGHT_SECONDARY_ROTATOR_PORT = 8;

  private static final int ENCODER_PORT = 1;

  // Motor and sensor inversions
  private static final boolean LEFT_PRIMARY_INVERTED = false;
  private static final boolean LEFT_SECONDARY_INVERTED = false;
  private static final boolean RIGHT_PRIMARY_INVERTED = false;
  private static final boolean RIGHT_SECONDARY_INVERTED = false;
  private static final boolean LEFT_PRIMARY_ROTATOR_INVERTED = false;
  private static final boolean LEFT_SECONDARY_ROTATOR_INVERTED = false;
  private static final boolean RIGHT_PRIMARY_ROTATOR_INVERTED = false;
  private static final boolean RIGHT_SECONDARY_ROTATOR_INVERTED = false;




    
}
