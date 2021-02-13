package frc.robot.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpiutil.math.Matrix;

/**
 * This class provides the odometry of a differential drive using Euler's Method (incrementing with time steps).
 */
public class DifferentialDriveOdometry {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private DifferentialDriveKinematics robot; 
    private Field2d field; 
    private Pose2d originalPose;
    private Matrix absoluteVelocity; //Matrix that stores translational and rotational velocities of the robot 
    private Rotation2d rotationamount;

    /**
     * Constructs a differential drive odometry object with the given necessitated inputs.
     * 
     * @param width: Width of the robot
     * @param wheelRadius: Radius of the wheels
     * @param xpos: X position of the wheel
     * @param ypos: Y position of the wheel
     * @param phi: Angular position of robot 
     */
    public DifferentialDriveOdometry(double width, double wheelRadius, double xpos, double ypos, double phi) 
    {
        //wR = wheelRadius;
        //a = width;
        field.setRobotPose(xpos, ypos, new Rotation2d(phi));
        originalPose = field.getRobotPose();
        robot = new DifferentialDriveKinematics(wheelRadius, width);

    }

    /**
     * Constructs a differential drive odometry object with less parameters and sets x and y position to 0.
     * 
     * @param width: width of the robot
     * @param wheelRadius: radius of the wheel
     * @param phi: given angle that the robot makes 
     */
    public DifferentialDriveOdometry(double width, double wheelRadius, double phi) 
    {
        field.setRobotPose(0, 0, new Rotation2d(phi));
        originalPose = field.getRobotPose();
        robot = new DifferentialDriveKinematics(wheelRadius, width);
    }


    /**
     * Using Euler's Method, this method provides an estimate of where the robot would be using the given inputs. 
     * 
     * @param deltaT: Change in time
     * @param vL: Angular velocity of left wheel
     * @param vR: Angular velocity of right wheel
     * @return Pose2d: Returns the estimated pose of the robot
     */
    public Pose2d getPosition(double deltaT, double endTime, double vL, double vR)
    {
        for(double time = 0; time < endTime; time+=deltaT)
        {
            absoluteVelocity = robot.getAbsoluteVelocity(vL, vR, field.getRobotPose().getRotation().getRadians());
            rotationamount = new Rotation2d(absoluteVelocity.get(2,0) * deltaT + field.getRobotPose().getRotation().getRadians());
            field.setRobotPose(absoluteVelocity.get(0,0) * deltaT + field.getRobotPose().getTranslation().getX(), absoluteVelocity.get(1,0) * deltaT + field.getRobotPose().getTranslation().getY(), rotationamount);
        }
        return field.getRobotPose();
    }

    //Reset the position of the robot to the original pose.
    public void resetPose()
    {
        field.setRobotPose(originalPose);
    }
}