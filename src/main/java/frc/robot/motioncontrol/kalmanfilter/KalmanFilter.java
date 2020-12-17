package frc.robot.motioncontrol.kalmanfilter;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {

    // class variables

    private SimpleMatrix x; // state vector
    private SimpleMatrix Q; // state (or process) noise covariance
    private SimpleMatrix P; // state covariance (accuracy of state)
    private SimpleMatrix z; // measurement
    private SimpleMatrix R; // measurement noise covariance
    private SimpleMatrix u; // control input vector
    private SimpleMatrix A; // relates previous state to current state, often found as F
    private SimpleMatrix B; // relates control input to current state
    private SimpleMatrix H; // relates current state to measurement
    private SimpleMatrix K; // process's Kalman gain

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
