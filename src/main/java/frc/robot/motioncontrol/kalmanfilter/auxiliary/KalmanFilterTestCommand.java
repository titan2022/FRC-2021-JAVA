package frc.robot.motioncontrol.kalmanfilter.auxiliary;

import java.util.Random;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motioncontrol.kalmanfilter.KalmanFilter;
import frc.wpilibjTemp.Field2d;

public class KalmanFilterTestCommand extends CommandBase {

    private KalmanFilter filter; // vector: [xpos, xvel, xacc, ypos, yvel, yacc]
    private Timer timer;
    private double t;
    private Field2d field;
    private Random rand;
    private double power;
    private Rotation2d theta;
    private SimpleMatrix u;
    private SimpleMatrix v;
    private SimpleMatrix posReal; // measurement format: [x,y] ^ T
    private SimpleMatrix posNoisy;
    private final double STDEV = 0.1;

    /**
     * Creates a KalmanFilterTestCommand object
     */

    public KalmanFilterTestCommand() {

        addRequirements();

    }

    /**
     * Initializes a Kalman filter test and all instance variables
     */

    @Override
    public void initialize() {

        // all instance variables
        
        rand = new Random();
        timer = new Timer();
        field = new Field2d();
        power = 0;
        theta = Rotation2d.fromDegrees(45);
        u = new SimpleMatrix(new double[][] { { 0 }, { 0 } });
        v = new SimpleMatrix(u);
        posReal = new SimpleMatrix(new double[][] { { 1 }, { 1 } });
        posNoisy = new SimpleMatrix(posReal);

        // putting power and theta so user can modify

        SmartDashboard.putNumber("Power", power);
        SmartDashboard.putNumber("Theta (Degs)", theta.getDegrees());
        updatePose();

        // see KalmanFilterTestCommandExplainer.pdf

        filter = new KalmanFilter(new SimpleMatrix(new double[][] { { 1 }, { 0 }, { 0 }, { 1 }, { 0 }, { 0 } }),
                SimpleMatrix.identity(6), SimpleMatrix.identity(6).scale(Math.pow(STDEV, 4)),
                SimpleMatrix.identity(2).scale(Math.pow(STDEV, 2)), updateA(),
                new SimpleMatrix(new double[][] { { 0, 0 }, { 0, 0 }, { 0.00001, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0.00001 } }),
                new SimpleMatrix(new double[][] { { 1, 0, 0, 0, 0, 0 }, { 0, 0, 0, 1, 0, 0 } }));

        timer.start();

    }

    /**
     * Periodic updating of Kalman filter test
     */

    @Override
    public void execute() {

        t = timer.get();
        timer.reset();

        // see KalmanFilterTestCommandExplainer.pdf

        filter.setA(updateA());

        // vi = v0 + at

        v = v.plus(u.scale(t));

        // getting user input for power/theta and converting to u vector

        power = SmartDashboard.getNumber("Power", 0);
        theta = Rotation2d.fromDegrees(SmartDashboard.getNumber("Theta (Degs)", -45));
        u.set(0, 0, (power * theta.getCos()));
        u.set(1, 0, (power * theta.getSin()));

        // getting real position of robot

        posReal.set(0, 0, field.getRobotPose().getTranslation().getX());
        posReal.set(1, 0, field.getRobotPose().getTranslation().getY());

        // adding Gaussian noise factor

        posNoisy.set(0, 0, (STDEV * rand.nextGaussian()) + posReal.get(0, 0));
        posNoisy.set(1, 0, (STDEV * rand.nextGaussian()) + posReal.get(1, 0));

        // running filter before putting numbers

        filter.runFilter(u, posNoisy);

        // adding numbers to SmartDashboard

        SmartDashboard.putNumber("x (real)", posReal.get(0, 0));
        SmartDashboard.putNumber("y (real)", posReal.get(1, 0));
        SmartDashboard.putNumber("x (noisy)", posNoisy.get(0, 0));
        SmartDashboard.putNumber("y (noisy)", posNoisy.get(1, 0));
        SmartDashboard.putNumber("x (filt.)", filter.getState().get(0, 0));
        SmartDashboard.putNumber("y (filt.)", filter.getState().get(3, 0));
        SmartDashboard.putNumber("vx (est.)", v.get(0, 0));
        SmartDashboard.putNumber("vy (est.)", v.get(1, 0));
        SmartDashboard.putNumber("vx (filt.)", filter.getState().get(1, 0));
        SmartDashboard.putNumber("vy (filt.)", filter.getState().get(4, 0));
        SmartDashboard.putNumber("ax (input)", u.get(0, 0));
        SmartDashboard.putNumber("ay (input)", u.get(1, 0));
        SmartDashboard.putNumber("ax (filt.)", filter.getState().get(2, 0));
        SmartDashboard.putNumber("ay (filt.)", filter.getState().get(5, 0));

        updatePose();

    }

    /**
     * End behavior of Kalman filter test
     */

    @Override
    public void end(boolean interrupted) {

        timer.stop();

    }

    /**
     * Updates pose of the robot.
     */

    private void updatePose() {

        // standard accel-based updating

        field.setRobotPose((posReal.get(0, 0) + t * v.get(0, 0) + Math.pow(t, 2) / 2 * u.get(0, 0)),
                (posReal.get(1, 0) + t * v.get(1, 0) + Math.pow(t, 2) / 2 * u.get(1, 0)), theta);

    }

    /**
     * Updates A matrix for specific time.
     * 
     * @return Updated A matrix.
     */

    private SimpleMatrix updateA() {

        // see KalmanFilterTestCommandExplainer.pdf

        return new SimpleMatrix(
                new double[][] { { 1, t, Math.pow(t, 2) / 2, 0, 0, 0 }, { 0, 1, t, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 },
                        { 0, 0, 0, 1, t, Math.pow(t, 2) / 2 }, { 0, 0, 0, 0, 1, t }, { 0, 0, 0, 0, 0, 0 } });

    }

}