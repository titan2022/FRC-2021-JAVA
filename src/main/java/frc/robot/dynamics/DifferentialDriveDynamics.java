package frc.robot.dynamics;

import frc.robot.Vector;
import org.ejml.simple.*;
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
     * @return linear acceleration of robot
     */
    public double getLinearAcceleration(Vector torques){
        //Lagrange model
        return ((1/(m*wR)) * (torques.getX() + torques.getY()));
    }

    /**
     * 
     * @param torques vector (right torque, left torque)
     * @return rotational acceleration of robot
     */
    public double getRotationalAcceleration(Vector torques){
        //Lagrange model
        return (((2*a)/(I*wR)) * (torques.getX() - torques.getY()));
    }
}