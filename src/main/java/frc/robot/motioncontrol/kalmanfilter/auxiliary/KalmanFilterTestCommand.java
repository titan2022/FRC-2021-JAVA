package frc.robot.motioncontrol.kalmanfilter.auxiliary;

import java.util.Random;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motioncontrol.kalmanfilter.KalmanFilter;

public class KalmanFilterTestCommand extends CommandBase {

    private Random rand;
    private double prevT;
    private double deltaT;

    public KalmanFilterTestCommand() {

        rand = new Random();

    }

    private double getNoisy(double real, double stdev) {

        return (stdev * rand.nextGaussian() + real);

    }

    private double getTime() {

        return (double) System.nanoTime() / 1E9;

    }

}