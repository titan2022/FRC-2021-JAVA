package frc.robot.motioncontrol.kalmanfilter.demo;

import org.ejml.simple.SimpleMatrix;
import java.util.Random;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.motioncontrol.kalmanfilter.CustomKalmanFilter;

public class KalmanFilterDemoCommand extends CommandBase {

    private Field2d field;
    private CustomKalmanFilter filter;
    private SimpleMatrix posReal;
    private SimpleMatrix posNoisy;
    private Rotation2d angle;
    private Random rand;
    private final SimpleMatrix ZERO_MATRIX = new SimpleMatrix(new double[][] { { 0 }, { 0 } });

    /**
     * Creates a KalmanFilterDemoCommand object
     */

    public KalmanFilterDemoCommand() {

        addRequirements();

    }

    /**
     * Initializes a Kalman filter demo and all instance variables
     */

    @Override
    public void initialize() {

        rand = new Random();
        field = new Field2d();
        angle = new Rotation2d();

        // starting position of robot on field

        posReal = new SimpleMatrix(new double[][] { { 5 }, { 5 } });
        posNoisy = new SimpleMatrix(posReal);
        field.setRobotPose(posReal.get(0, 0), posReal.get(1, 0), angle);

        // initializing KalmanFilter, Q = 0.0001I, R = 0.25I, P = A = B = H = I, I in M2
        // Q is generally very small (process noise)
        // R is generally I * variance, or I * (stdev^2) = 0.25I

        filter = new CustomKalmanFilter(new SimpleMatrix(posReal), SimpleMatrix.identity(2),
                SimpleMatrix.identity(2).scale(0.0001), SimpleMatrix.identity(2).scale(0.25), SimpleMatrix.identity(2),
                SimpleMatrix.identity(2), SimpleMatrix.identity(2));

    }

    /**
     * Periodic updating of Kalman filter demo
     */

    @Override
    public void execute() {

        // getting real position of robot

        posReal.set(0, 0, field.getRobotPose().getTranslation().getX());
        posReal.set(1, 0, field.getRobotPose().getTranslation().getY());

        // adding Gaussian noise factor with avg = 0, stdev = 0.5

        posNoisy.set(0, 0, (0.5 * rand.nextGaussian()) + posReal.get(0, 0));
        posNoisy.set(1, 0, (0.5 * rand.nextGaussian()) + posReal.get(1, 0));

        filter.runFilter(ZERO_MATRIX, new SimpleMatrix(posNoisy));

        // adding numbers to SmartDashboard

        SmartDashboard.putNumber("x (real)", posReal.get(0, 0));
        SmartDashboard.putNumber("y (real)", posReal.get(1, 0));
        SmartDashboard.putNumber("x (noisy)", posNoisy.get(0, 0));
        SmartDashboard.putNumber("y (noisy)", posNoisy.get(1, 0));
        SmartDashboard.putNumber("x (est.)", filter.getState().get(0, 0));
        SmartDashboard.putNumber("y (est.)", filter.getState().get(1, 0));

    }

}