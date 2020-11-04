package frc.robot.motioncontrol.kalmanfilter;

public class KalmanFilter {

    // class variables, x is state vector not position

    private double tDelta;
    private double[][] x;
    private double aNoise;
    private double a;
    private double[][] transitionMatrix;
    private double[][] inputMatrix;
    private double[][] measMatrix;
    private double measNoise;
    
}
