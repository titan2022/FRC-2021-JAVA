package frc.robot.motioncontrol.kalmanfilter;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {

    // class variables

    private SimpleMatrix x; // state vector
    private SimpleMatrix Q; // state (or process) noise covariance
    private SimpleMatrix P; // state covariance (accuracy of state)
    private SimpleMatrix R; // measurement noise covariance
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

    /**
     * Predicts future state (x)
     * @param u - Control input from user.
     */

    private void predictState(SimpleMatrix u) {

        x = (A.mult(x)).plus(B.mult(u));

    }

    /**
     * Predicts state covariance (P)
     */

    private void predictCovariance() {

        P = (A.mult(P).mult(A.transpose())).plus(Q);

    }

    /**
     * Predits Kalman filter's state and covariance.
     * @param u - Control input from user.
     */

    public void predictFilter(SimpleMatrix u) {

        predictState(u);
        predictCovariance();

    }

    /**
     * Updates Kalman gain (K)
     */

    private void updateKalmanGain() {

        K = P.mult(H.transpose()).mult(((H.mult(P).mult(H.transpose())).plus(R)).invert());

    }

    /**
     * Updates current state (x)
     * @param z - Measurement from system.
     */

    private void updateState(SimpleMatrix z) {

        x = x.plus(K.mult(z.minus(H.mult(x))));

    }

    /**
     * Updates state covariance (P)
     */

    private void updateCovariance() {

        P = (SimpleMatrix.identity(x.numRows()).minus(K.mult(H))).mult(P);

    }

    /**
     * Updates Kalman filter's state, covariance, and Kalman gain.
     * @param z - Measurement from system.
     */

    public void updateFilter(SimpleMatrix z) {

        updateKalmanGain();
        updateState(z);
        updateCovariance();

    }

    /**
     * Returns current state (predicted if after predict step)
     * @return Current state vector.
     */

    public SimpleMatrix getState() {

        return x;

    }

    /**
     * Returns current covariance (predicted if after predict step)
     * @return Current state covariance.
     */

    public SimpleMatrix getCovariance() {

        return P;

    }

}
