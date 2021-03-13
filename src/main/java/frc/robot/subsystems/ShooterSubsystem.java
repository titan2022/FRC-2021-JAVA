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

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import edu.wpi.first.wpilibj.examples.armbotoffboard.Constants.ArmConstants;
import edu.wpi.first.wpilibj.examples.armbotoffboard.ExampleSmartMotorController;

/**
 * @author Abhi
 * @author Deepu
 */
public class ShooterSubsystem extends SubsystemBase{

     // Physical parameters
  public static final double ROBOT_TRACK_WIDTH = 26.75/39.37; // meter
  public static final double WHEEL_RADIUS = 6/39.37; // meters
  public static final double ENCODER_TICKS = 4096; // Ticks/rotation of CTREMagEncoder
  private static final double ANGLE_TO_TICK =  1 / (360 * ENCODER_TICKS); //temp value
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
    private double rpsBall;
    private double rpsTalon;
    private double xCoord;
    private double yCoord;
    private double zCoord;
    private double targetXCoord;
    private double targetYCoord;
    private double targetZCoord;
    private double radius;
    private double g; 
    private double t;
    private double distance;
    private double height;
    // measure the current linear velocity of the wheel, then assume that's starting velocity. plug it back in to find the least
    // amount of time, optimize time in respect to theta
    public Shooter()
    {   
        g = -9.8;
        t = 10;
        distance = targetXCoord - xCoord;
        height = targetYCoord - yCoord;
    }
   
    //First set the speed of the motor and then calculate the rpm and acceleration of the ball.
    public double calcRPSBall()
    {
        double speed = 8.0;
        leftPrimary.set(speed);
        leftSecondary.set(speed);
        rightPrimary.set(speed);
        rightSecondary.set(speed);
        rpsBall = speed / radius / (2 * Math.PI);
        return rpsBall;
    }

    //First set the speed of the motor and then calculate the rpm and acceleration of the wheel.
    public double getRPSTalon()
    {
        double speed = 8.0;
        leftPrimary.set(speed);
        leftSecondary.set(speed);
        rightPrimary.set(speed);
        rightSecondary.set(speed);
        rpsTalon = speed / radius / (2 * Math.PI);
        return rpsTalon;
    }

    public double getTargetXCoord()
    {
        return targetXCoord;
    }

    public double getTargetYCoord()
    {
        return targetYCoord;
    }

    public double getTargetZCoord()
    {
        return targetZCoord;
    }

    public double getXCoord()
    {
        return xCoord;
    }

    public double getYCoord()
    {
        return yCoord;
    }

    public double getZCoord()
    {
        return zCoord;
    }


    //gets the angle of the hood
    public double getAngleToTargetH()
    {
        LimelightMath vision = new LimelightMath();
        return vision.calculateAngleToTargetH();
    }

    //gets the angle of the hood
    public double getHoodAngle()
    {
        double angle = leftPrimary.getSelectedSensorPosition();
        return angle;
    }

    public double getAngleToTargetY()
    {
        LimelightMath vision = new LimelightMath();
        return vision.calculateAngleToTargetV();
    }

    public void setHoodYAngle(double angle)
    {
        leftPrimary.set(ControlMode.MotionProfile, angle *  ANGLE_TO_TICK);
        leftSecondary.set(ControlMode.MotionProfile, angle *  ANGLE_TO_TICK);
        rightPrimary.set(ControlMode.MotionProfile, angle *  ANGLE_TO_TICK);
        rightSecondary.set(ControlMode.MotionProfile, angle *  ANGLE_TO_TICK);
    }

    

    //Calculates velocity of the shooter based on the hood angle.
    public double getShooterVelocityX()
    {
        Shooter shooter = new Shooter();
        double initialAngle = shooter.getHoodAngle();
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
        double initialAngle = shooter.getHoodAngle();
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
        // covert velocity to ticks/ minute (i think)
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

    public double getMotorAngularAcceleration()
    {
        double angularAcceleration = getShooterFinalVelocity()/time;
        return angularAcceleration;
        //convert the angular acceleration to ticks per minute^2 
    }

    public void setOutput(ControlMode mode, double leftOutputValue, double rightOutputValue) {

        if (mode == ControlMode.Velocity) {
    
          if (leftOutputValue > MAX_SPEED) {
            leftOutputValue = MAX_SPEED;
          }
    
          if (rightOutputValue > MAX_SPEED) {
            rightOutputValue = MAX_SPEED;
          }
    
        }
    
        // TODO: is check the current usage from Power Subsystem to restrict overcurrent
        leftPrimary.set(mode, leftOutputValue);
        rightPrimary.set(mode, rightOutputValue);
      }
    
    public double getExitAngle()
    {
        double exitAngle = 0;
        exitAngle = Math.atan((height + (1/2 * g* t * t))/distance);
        return exitAngle;
    }

    public double getInitialVelocity()
    {
        double initialVelocity = 0;
        initialVelocity = (distance/Math.cos(getExitAngle())) * t;
        return initialVelocity;
    }

    public void setTalonVelocity()
    {
        rightPrimary.set(getInitialVelocity());
    }



    

}
