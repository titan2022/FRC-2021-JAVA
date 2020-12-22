package frc.robot.motioncontrol.kalmanfilter;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KalmanFilter extends SubsystemBase {

    // class variables

    private SimpleMatrix x; // state vector
    private SimpleMatrix P; // state covariance (accuracy of state)
    private SimpleMatrix K; // process's Kalman gain
    private SimpleMatrix Q; // state (or process) noise covariance
    private SimpleMatrix R; // measurement noise covariance
    private SimpleMatrix A; // relates previous state to current state (otherwise seen as F)
    private SimpleMatrix B; // relates control input to current state
    private SimpleMatrix H; // relates measurement to current state

    /**
     * Creates a new KalmanFilter
     * 
     * @param x - Initial state vector.
     * @param P - Initial state covariance.
     * @param Q - Process noise covariance.
     * @param R - Measurement noise covariance.
     * @param A - Relates previous state to current state.
     * @param B - Relates control input to current state.
     * @param H - Relates measurement to current state.
     */

    public KalmanFilter(SimpleMatrix x, SimpleMatrix P, SimpleMatrix Q, SimpleMatrix R, SimpleMatrix A, SimpleMatrix B,
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
     * Gets B, which relates control input to current state.
     * 
     * @return Current B matrix.
     */

    public SimpleMatrix getB() {

        return B;

    }

    /**
     * Gets H, which relates measurement to current state.
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
     * Sets B, which relates control input to current state.
     * 
     * @param B - B matrix.
     */

    public void setB(SimpleMatrix B) {

        this.B = B;

    }

    /**
     * Sets H, which relates measurement to current state.
     * 
     * @param H - H matrix.
     */

    public void setH(SimpleMatrix H) {

        this.H = H;

    }

}
