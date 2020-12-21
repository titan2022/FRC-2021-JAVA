package frc.robot.motioncontrol.kalmanfilter.auxiliary;

import java.util.Random;

import org.ejml.simple.SimpleMatrix;

import frc.robot.motioncontrol.kalmanfilter.KalmanFilter;

public class KalmanFilterTestCommand {

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