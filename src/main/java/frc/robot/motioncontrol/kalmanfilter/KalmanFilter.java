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

    public KalmanFilter(SimpleMatrix x, SimpleMatrix Q, SimpleMatrix P, SimpleMatrix R, SimpleMatrix A, SimpleMatrix B, SimpleMatrix H) {

        this.x = x;
        this.Q = Q;
        this.P = P;
        this.R = R;
        this.A = A;
        this.B = B;
        this.H = H;

    }

    private void updateKalmanGain() {

        K = P.mult(H.transpose()).mult(((H.mult(P).mult(H.transpose())).plus(R)).invert());

    }

    private void updateState() {

        x = x.plus(K.mult(z.minus(H.mult(x))));

    }

    private void updateCovariance() {

        P = (SimpleMatrix.identity(x.numRows()).minus(K.mult(H))).mult(P);

    }

}
