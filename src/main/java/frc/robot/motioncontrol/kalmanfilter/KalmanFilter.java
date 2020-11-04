package frc.robot.motioncontrol.kalmanfilter;

import frc.robot.motioncontrol.MatrixOperations;

public class KalmanFilter {

    // class variables

    private double tDelta;
    private double a; // given acceleration
    private double aNoise;
    private double measNoise; // measurement noise
    private double sz; // measurement noise covariance
    private double[][] x; // state vector
    private double[][] transMatrix; // transition matrix
    private double[][] inputMatrix;
    private double[][] measMatrix; // measurement matrix
    private double[][] sw; // process noise covariance
    private double[][] P; // estimation covariance

    public KalmanFilter(double tDelta, double a, double aNoise, double measNoise) {

        this.tDelta = tDelta;
        this.a = a;
        this.aNoise = aNoise;
        this.measNoise = measNoise;
        transMatrix = new double[][] { { 1, tDelta }, { 0, 1 } };
        inputMatrix = new double[][] { { Math.pow(tDelta, 2) / 2 }, { tDelta } };
        measMatrix = new double[][] { { 1, 0 } };
        x = new double[][] { { 0 }, { 0 } };
        sz = Math.pow(measNoise, 2);
        sw = MatrixOperations.scalarMultiply(Math.pow(aNoise, 2),
                new double[][] { { Math.pow(tDelta, 4) / 4, Math.pow(tDelta, 3) / 2 },
                        { Math.pow(tDelta, 3) / 2, Math.pow(tDelta, 2) } });
        P = sw.clone();

    }

}
