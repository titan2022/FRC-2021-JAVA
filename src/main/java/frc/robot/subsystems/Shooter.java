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
import frc.robot.vision.LimelightMath;
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
   
    //First set the speed of the motor and then calculate the rpm and acceleration of the wheel.
    public double calcRPMBall()
    {
        double speed = 8.0;
        leftPrimary.set(speed);
        leftSecondary.set(speed);
        rightPrimary.set(speed);
        rightSecondary.set(speed);
        rpm = speed / 60 / radius / (2 * Math.PI);
        return rpm;
    }

    //First set the speed of the motor and then calculate the rpm and acceleration of the wheel.
    public double calcRPMTalon()
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
    public double getHoodXAngle()
    {
        double angle = 45.0;
        return angle;
    }

    //gets the angle of the hood
    public double getAngleToTargetH()
    {
        LimelightMath vision = new LimelightMath();
        return vision.calculateAngleToTargetH();
    }

    //gets the angle of the hood
    public double getHoodYAngle()
    {
        double angle = 45.0;
        return angle;
    }

    //gets the angle of the hood
    public double getAngleToTargetY()
    {
        LimelightMath vision = new LimelightMath();
        return vision.calculateAngleToTargetV();
    }

    //Calculates velocity of the shooter based on the hood angle.
    public double getShooterVelocityX()
    {
        Shooter shooter = new Shooter();
        double initialAngle = shooter.getHoodYAngle();
        
       

        LimelightMath vision = new LimelightMath();
        double distanceToTargetX = vision.calculateDistance();
        double desiredTime = 5.0;
        double shooterVelocity = distanceToTargetX/(Math.cos(initialAngle) * desiredTime);
        return shooterVelocity;

    }

    //Calculates velocity of the shooter based on the hood angle.
    public double getShooterVelocityY()
    {
        Shooter shooter = new Shooter();
        double initialAngle = shooter.getHoodYAngle();
        double accelerationY = -9.8;
        double desiredTime = 5.0;

        LimelightMath vision = new LimelightMath();
        double distanceY = vision.calculateDistance() * Math.tan(vision.calculateAngleToTargetV());
        double shooterVelocity = (distanceY + 0.5 * accelerationY * desiredTime * desiredTime) / (Math.sin(initialAngle)* desiredTime);
        return shooterVelocity;
    }

    //Calculate Final Velocity relative to Angle of Hood.
    public double getShooterFinalVelocity()
    {
        Shooter shooter = new Shooter();
        double finalVelocity = Math.sqrt(Math.pow(shooter.getShooterVelocityX(), 2) + Math.pow(shooter.getShooterVelocityY(), 2));
        return finalVelocity;  
    }

    public void setMotorsSpeed()
    {
        Shooter shooter = new Shooter();
        double speed = shooter.getShooterFinalVelocity();
        leftPrimary.set(speed);
        leftSecondary.set(speed);
        rightPrimary.set(speed);
        rightSecondary.set(speed);
    }

    
    

    

}
