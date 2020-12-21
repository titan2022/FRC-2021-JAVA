package frc.robot.motioncontrol.kalmanfilter.auxiliary;

import java.util.Random;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motioncontrol.kalmanfilter.KalmanFilter;

public class KalmanFilterTestCommand extends CommandBase {

    private KalmanFilter filter;
    private Timer timer;
    private Random rand;
    private double prevT;
    private double deltaT;

    public KalmanFilterTestCommand() {

        addRequirements();

    }
    
    @Override
    public void initialize() {

        rand = new Random();
        timer = new Timer();
        timer.start();

    }

    private double getNoisy(double actual, double stdev) {

        return (stdev * rand.nextGaussian() + actual);

    }

}