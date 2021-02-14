package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
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
    private VictorSPX victory;
    private double radius;
   
    //First set the speed of the motor and then calculate the rpm and acceleration of the wheel.
    private double calcRPM()
    {

        return 0;
    }

    //gets the angle of the hood
    private void getHoodAngle()
    {

    }

    //Finds the exitAngle of the ball based on the angle of the hood.
    private double getExitAngle()
    {
        return 0;
    }
    
    //Finds the targetCoords based on the positioning of the shooter. 
    private void targetCoords()
    {

    }

    //Find the trajectory of the ball based on the initial and final coords.
    private void findTrajectory()
    {

    }

    //Shoots the ball
    private void shoot()
    {

    }

    

}
