package frc.robot.motioncontrol.kalmanfilter;

import org.ejml.simple.SimpleMatrix;
import java.util.Random;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.wpilibjTemp.Field2d;

public class KalmanFilterDemoCommand extends CommandBase {

    private Field2d field;
    private KalmanFilter filter;
    private SimpleMatrix posReal;
    private SimpleMatrix posNoisy;
    private Rotation2d angle;
    private Random rand;

    public KalmanFilterDemoCommand() {

        addRequirements();

    }

    @Override
    public void initialize() {

        rand = new Random();
        field = new Field2d();
        angle = new Rotation2d();

        posReal = new SimpleMatrix(new double[][] { { 0 }, { 0 } });
        field.setRobotPose(posReal.get(0, 0), posReal.get(1, 0), angle);
        posNoisy = new SimpleMatrix(posReal);

        SimpleMatrix Q = SimpleMatrix.identity(2).scale(0.0001);
        SimpleMatrix R = SimpleMatrix.identity(2).scale(0.01);

        filter = new KalmanFilter(new SimpleMatrix(posReal), Q, SimpleMatrix.identity(2), R, SimpleMatrix.identity(2),
                SimpleMatrix.identity(2), SimpleMatrix.identity(2));

    }

    @Override
    public void execute() {

        posReal.set(0, 0, field.getRobotPose().getTranslation().getX());
        posReal.set(1, 0, field.getRobotPose().getTranslation().getY());

        posNoisy.set(0, 0, (1 + 0.1 * rand.nextGaussian()) * posReal.get(0, 0));
        posNoisy.set(1, 0, (1 + 0.1 * rand.nextGaussian()) * posReal.get(1, 0));

        filter.runFilter(new SimpleMatrix(new double[][] { { 0 }, { 0 } }), posNoisy);

        SmartDashboard.putNumber("x (real)", posReal.get(0, 0));
        SmartDashboard.putNumber("y (real)", posReal.get(1, 0));
        SmartDashboard.putNumber("x (noisy)", posNoisy.get(0, 0));
        SmartDashboard.putNumber("y (noisy)", posNoisy.get(1, 0));
        SmartDashboard.putNumber("x (est.)", filter.getState().get(0, 0));
        SmartDashboard.putNumber("y (est.)", filter.getState().get(1, 0));

    }

}