package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * @author Abhi
 * @author Deepu
 * @author Hari
 */
public class Shooter extends SubsystemBase{

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

  private static final int ENCODER_PORT = 0;

  // Motor and sensor inversions
  private static final boolean LEFT_PRIMARY_INVERTED = false;
  private static final boolean LEFT_SECONDARY_INVERTED = false;
  private static final boolean RIGHT_PRIMARY_INVERTED = false;
  private static final boolean RIGHT_SECONDARY_INVERTED = false;
  private static final boolean LEFT_PRIMARY_MOTOR_SENSOR_PHASE = false;
  private static final boolean RIGHT_PRIMARY_MOTOR_SENSOR_PHASE = false;

  // Physical limits
  private static final double MAX_SPEED = 10; // meters/sec
  private static final int PEAK_CURRENT_LIMIT = 60;
  private static final int CONTINUOUS_CURRENT_LIMIT = 50;

  // Phoenix Physics Sim Variables
  private static final double FULL_ACCEL_TIME = 0.75; // sec
  private static final double MAX_MOTOR_VEL = 4000; // ticks/(100ms)

  // Physical and Simulated Hardware
  // These talon objects are also simulated
  private static final WPI_TalonSRX leftPrimary = new WPI_TalonSRX(LEFT_PRIMARY_PORT)
    , leftSecondary = new WPI_TalonSRX(LEFT_SECONDARY_PORT)
    , rightPrimary = new WPI_TalonSRX(RIGHT_PRIMARY_PORT)
    , rightSecondary = new WPI_TalonSRX(RIGHT_SECONDARY_PORT);

    private double time;
    private double velocity;
    private double rpm;
    private double xCoord;
    private double yCoord;
    private double zCoord;
    private double targetXCoord;
    private double targetYCoord;
    private double targetZCoord; 
    private double exitAngle;
    private TalonSRX talon;
    private VictorSPX victor;
    private double radius;
 
    public Shooter(TalonSRXConfiguration leftConfig, TalonSRXConfiguration rightConfig)
    {
        
    }

    //First set the speed of the motor and then calculate the rpm and acceleration of the wheel.
    public double calcRPM()
    {
        double speed = 8.0;
        leftPrimary.set(speed);
        leftSecondary.set(speed);
        rightPrimary.set(speed);
        rightSecondary.set(speed);
        rpm = speed / 60 / radius / (2 * Math.PI);
        return rpm;
    }

    //gets the angle of the hood
    public void getHoodAngle()
    {
        

    }

    //Finds the exitAngle of the ball based on the angle of the hood.
    public double getExitAngle()
    {
        return 0;
    }
    
    //Finds the targetCoords based on the positioning of the shooter. 
    public void targetCoords()
    {

    }

    //Find the trajectory of the ball based on the initial and final coords.
    public void findTrajectory()
    {

    }

    

    

}
