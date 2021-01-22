package frc.robot.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.kinematics.*;
import edu.wpi.first.wpiutil.math.Matrix;

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
    public DifferentialDriveOdometry(double width, double wheelRadius, double xpos, double ypos, double phi) {
        //wR = wheelRadius;
        //a = width;
        f.setRobotPose(xpos, ypos, new Rotation2d(phi));
        pose = f.getRobotPose();
        object = new DifferentialDriveKinematics(wheelRadius, width);

    }
    /**
     * 
     * @param deltaT: Change in time
     * @param vL: Angular velocity of left wheel
     * @param vR: Angular velocity of right wheel
     * @return Pose2d: Returns the estimated pose of the robot
     */
    public Pose2d getPosition(double deltaT, double vL, double vR)
    {
        for(double i = 0; i < deltaT; i+=0.01)
        {
            absoluteVelocity = object.getAbsoluteVelocity(vL, vR, f.getRobotPose().getRotation().getRadians());
            rotationamount = new Rotation2d(absoluteVelocity.get(2,0) * i + f.getRobotPose().getRotation().getRadians());
            f.setRobotPose(absoluteVelocity.get(0,0) * i + f.getRobotPose().getTranslation().getX(), absoluteVelocity.get(1,0) * i + f.getRobotPose().getTranslation().getY(), rotationamount);
        }
        return f.getRobotPose();
    }
}