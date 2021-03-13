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
  public static final double ENCODER_TICKS = 4096; // Ticks/rotation of CTREMagEncoder
  private static final double ANGLE_TO_TICK =  1 / (360 * ENCODER_TICKS); //temp value
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
     , rightPrimary = new WPI_TalonSRX(RIGHT_PRIMARY_PORT);

    private double xCoord;
    private double yCoord;
    private double targetXCoord;
    private double targetYCoord;
    private double g; 
    private double t;
    private double distance;
    private double height;
    // measure the current linear velocity of the wheel, then assume that's starting velocity. plug it back in to find the least
    // amount of time, optimize time in respect to theta
    public ShooterSubsystem()
    {   
        g = -9.8;
        t = 10;
        distance = targetXCoord - xCoord;
        height = targetYCoord - yCoord;
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

    public double optimizeAngle()
    {
        double initialAngleMax = 0.0;
        initialAngleMax = Math.atan(height/distance);
        return initialAngleMax;
    }

    public void setTalonVelocity()
    {
        rightPrimary.set(getInitialVelocity());
    }
   

}
