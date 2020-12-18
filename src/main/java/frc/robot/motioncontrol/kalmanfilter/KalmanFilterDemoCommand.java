package frc.robot.motioncontrol.kalmanfilter;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.wpilibjTemp.Field2d;

public class KalmanFilterDemoCommand extends CommandBase {

    private Field2d field;
    private KalmanFilter filter;

    public KalmanFilterDemoCommand() {

        addRequirements();

    }

    @Override
    public void initialize() {

        field = new Field2d();
        SimpleMatrix x = new SimpleMatrix(new double[][] { { 0 }, { 0 } });
        SimpleMatrix Q = SimpleMatrix.identity(2).scale(0.00001);
        SimpleMatrix R = SimpleMatrix.identity(2).scale(0.001);

        filter = new KalmanFilter(x, Q, SimpleMatrix.identity(2), R, SimpleMatrix.identity(2), SimpleMatrix.identity(2),
                SimpleMatrix.identity(2));

    }

}