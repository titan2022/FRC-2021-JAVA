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
    private SimpleMatrix posHat;
    private Rotation2d angle;
    private Random rand;

    public KalmanFilterDemoCommand() {

        addRequirements();

    }

    @Override
    public void initialize() {

        field = new Field2d();
        angle = new Rotation2d();
        posReal = new SimpleMatrix(new double[][] { { 0 }, { 0 } });
        posHat = new SimpleMatrix(posReal);
        SimpleMatrix Q = SimpleMatrix.identity(2).scale(0.00001);
        SimpleMatrix R = SimpleMatrix.identity(2).scale(0.001);

        filter = new KalmanFilter(posHat, Q, SimpleMatrix.identity(2), R, SimpleMatrix.identity(2), SimpleMatrix.identity(2),
                SimpleMatrix.identity(2));

    }

    @Override
    public void execute() {

        field.setRobotPose(posReal.get(0, 0), posReal.get(1, 0), angle);
        SmartDashboard.putNumber("x (real)", field.getRobotPose().getTranslation().getX());
        SmartDashboard.putNumber("y (real)", field.getRobotPose().getTranslation().getY());


    }

}