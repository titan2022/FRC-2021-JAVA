package frc.robot.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.kinematics.*;
import frc.wpilibjTemp.Field2d;
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
     * @param width
     * @param wheelRadius
     * @param xpos
     * @param ypos
     * @param phi
     */
    public DifferentialDriveOdometry(double width, double wheelRadius, double xpos, double ypos, double phi) {
        //wR = wheelRadius;
        //a = width;
        f.setRobotPose(xpos, ypos, new Rotation2d(phi));
        pose = f.getRobotPose();
        object = new DifferentialDriveKinematics(wheelRadius, width);

        //DifferentialJacobian = object.getAbsoluteVelocity(vL, vR, f.getRobotPose().getRotation().getRadians());
    }
    /**
     * 
     * @param deltaT
     * @param vL
     * @param vR
     * @return
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