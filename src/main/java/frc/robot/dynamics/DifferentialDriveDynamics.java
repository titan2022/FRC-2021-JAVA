package frc.robot.dynamics;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Vector;

/**
 * 
 */
public class DifferentialDriveDynamics {
    //wR - wheel radius
    //a - half of the width of the robot
    //I - moment of inertia around center of gravity
    //tL, tR - torque of left and right sides
    private double wR, a, I, m;

    /**
     * 
     * @param wheelRadius 
     * @param robotWidth distance between opposite wheels
     * @param moment_of_inertia
     * @param mass in kg
     */
    public DifferentialDriveDynamics(double wheelRadius, double robotWidth, double moment_of_inertia, double mass){

        wR = wheelRadius;
        a = robotWidth/2;
        I = moment_of_inertia;
        m = mass;
    }

    /**
     * 
     * @param torques vector (right torque, left torque)
     * @return linear velocity of robot
     */
    public double getLinearVelocity(Vector torques){
        return ((1/(m*wR)) * (torques.getX() + torques.getY()));
    }

    /**
     * 
     * @param torques vector (right torque, left torque)
     * @return rotational velocity of robot
     */
    public double getRotationalVelocity(Vector torques){
        return (((2*a)/(I*wR)) * (torques.getX() - torques.getY()));
    }
}