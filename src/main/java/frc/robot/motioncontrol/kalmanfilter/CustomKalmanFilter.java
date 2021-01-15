package frc.robot.motioncontrol.kalmanfilter;

import org.ejml.simple.SimpleMatrix;

public class CustomKalmanFilter {

    // class variables

    // format: [matrix name/declaration]; // [description] (dimensions)
    // z is the measurement vector (m x 1)
    // u is the input vector (L x 1)
    private SimpleMatrix x; // state vector (n x 1)
    private SimpleMatrix P; // state covariance (n x n)
    private SimpleMatrix K; // process Kalman gain (n x m)
    private SimpleMatrix Q; // process noise covariance (n x n)
    private SimpleMatrix R; // measurement noise covariance (m x m)
    private SimpleMatrix A; // relates previous state to current state (n x n)
    private SimpleMatrix B; // relates control input to current state (n x L)
    private SimpleMatrix H; // relates current state to measurement (m x n)

    /**
     * Creates a new KalmanFilter
     * 
     * @param x - Initial state vector (n x 1).
     * @param P - Initial state covariance (n x n).
     * @param Q - Process noise covariance (n x n).
     * @param R - Measurement noise covariance (m x m).
     * @param A - Relates previous state to current state (n x n).
     * @param B - Relates control input to current state (n x L).
     * @param H - Relates current state to measurement (m x n).
     */

    public CustomKalmanFilter(SimpleMatrix x, SimpleMatrix P, SimpleMatrix Q, SimpleMatrix R, SimpleMatrix A, SimpleMatrix B,
            SimpleMatrix H) {

        this.x = x;
        this.P = P;
        this.Q = Q;
        this.R = R;
        this.A = A;
        this.B = B;
        this.H = H;

    }

    /**
     * Predicts future state (x).
     * 
     * @param u - Control input from user.
     */

    private void predictState(SimpleMatrix u) {

        x = (A.mult(x)).plus(B.mult(u));

    }

    /**
     * Predicts state covariance (P).
     */

    private void predictCovariance() {

        P = (A.mult(P).mult(A.transpose())).plus(Q);

    }

    /**
     * Predits Kalman filter's state and covariance.
     * 
     * @param u - Control input from user.
     */

    public void predictFilter(SimpleMatrix u) {

        predictState(u);
        predictCovariance();

    }

    /**
     * Updates Kalman gain (K).
     */

    private void updateKalmanGain() {

        K = P.mult(H.transpose()).mult(((H.mult(P).mult(H.transpose())).plus(R)).invert());

    }

    /**
     * Updates current state (x).
     * 
     * @param z - Measurement from system.
     */

    private void updateState(SimpleMatrix z) {

        x = x.plus(K.mult(z.minus(H.mult(x))));

    }

    /**
     * Updates state covariance (P).
     */

    private void updateCovariance() {

        P = (SimpleMatrix.identity(x.numRows()).minus(K.mult(H))).mult(P);

    }

    /**
     * Updates Kalman filter's state, covariance, and Kalman gain.
     * 
     * @param z - Measurement from system.
     */

    public void updateFilter(SimpleMatrix z) {

        updateKalmanGain();
        updateState(z);
        updateCovariance();

    }

    /**
     * Convenience method that predicts on input then updates on measurement.
     * 
     * @param u - Control input from user.
     * @param z - Measurement from system.
     */

    public void runFilter(SimpleMatrix u, SimpleMatrix z) {

        predictFilter(u);
        updateFilter(z);

    }

    /**
     * Returns current state (predicted if after predict step).
     * 
     * @return Current state vector.
     */

    public SimpleMatrix getState() {

        return x;

    }

    /**
     * Returns current covariance (predicted if after predict step).
     * 
     * @return Current state covariance.
     */

    public SimpleMatrix getCovariance() {

        return P;

    }

    /**
     * Gets A, which relates previous state to current state.
     * 
     * @return Current A matrix.
     */

    public SimpleMatrix getA() {

        return A;

    }

    /**
     * Gets H, which relates current state to measurement.
     * 
     * @return Current H matrix.
     */

    public SimpleMatrix getH() {

        return H;

    }

    /**
     * Sets A, which relates previous state to current state.
     * 
     * @param A - A matrix.
     */

    public void setA(SimpleMatrix A) {

        this.A = A;

    }

    /**
     * Sets H, which relates current state to measurement.
     * 
     * @param H - H matrix.
     */

    public void setH(SimpleMatrix H) {

        this.H = H;

    }

}
