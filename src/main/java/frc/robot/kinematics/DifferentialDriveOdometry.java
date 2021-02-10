package frc.robot.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.kinematics.*;
import edu.wpi.first.wpiutil.math.Matrix;

//This class represents the odometry of a differential drive using Euler's Method. It will be compared against that of WPILib's, which doesn't use Euler's Method and instead uses 
//Twist3D.

public class DifferentialDriveOdometry {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    //private double wR, a;
    private DifferentialDriveKinematics object; 
    private Field2d f; 
    private Pose2d pose;
    private Matrix absoluteVelocity;
    private Rotation2d rotationamount;
    /**
     * 
     * @param width: Width of the robot
     * @param wheelRadius: Radius of the wheels
     * @param xpos: X position of the wheel
     * @param ypos: Y position of the wheel
     * @param phi: Angular position of robot 
     */
    public DifferentialDriveOdometry(double width, double wheelRadius, double xpos, double ypos, double phi) 
    {
        f = new Field2d();
        f.setRobotPose(xpos, ypos, new Rotation2d(phi));
        pose = f.getRobotPose();
        object = new DifferentialDriveKinematics(wheelRadius, width);
    }
    /**
     * 
     * @param endTime: Change in time
     * @param vL: Angular velocity of left wheel
     * @param vR: Angular velocity of right wheel
     * @return Pose2d: Returns the estimated pose of the robot
     */
    public Pose2d getPosition(double endTime, double vL, double vR)
    {
        double deltaT = 0.001;
        for(double time = 0; time < endTime; time += deltaT)
        {
            absoluteVelocity = object.getAbsoluteVelocity(vL, vR, f.getRobotPose().getRotation().getRadians());
            rotationamount = new Rotation2d(absoluteVelocity.get(2,0) * deltaT + f.getRobotPose().getRotation().getRadians());
            f.setRobotPose(absoluteVelocity.get(0,0) * deltaT + f.getRobotPose().getTranslation().getX(), absoluteVelocity.get(1,0) * deltaT + f.getRobotPose().getTranslation().getY(), rotationamount);
        }
        return f.getRobotPose();
    }
}