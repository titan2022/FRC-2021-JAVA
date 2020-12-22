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
    private SimpleMatrix posReal; // measurement format: [x,y] ^ T
    private SimpleMatrix prevPosReal;
    private SimpleMatrix posNoisy;
    private final double STDEV = 0.25;

    public KalmanFilterTestCommand() {

        addRequirements();

    }

    @Override
    public void initialize() {

        rand = new Random();
        timer = new Timer();
        field = new Field2d();
        power = 0;
        theta = Rotation2d.fromDegrees(45);
        u = new SimpleMatrix(new double[][] { { 0 }, { 0 } });
        posReal = new SimpleMatrix(new double[][] { { 1 }, { 1 } });
        posNoisy = new SimpleMatrix(posReal);
        SmartDashboard.putNumber("Power", power);
        SmartDashboard.putNumber("Theta (Degs)", theta.getDegrees());
        updatePose();

        filter = new KalmanFilter(new SimpleMatrix(new double[][] { { 1 }, { 0 }, { 0 }, { 1 }, { 0 }, { 0 } }),
                SimpleMatrix.identity(6), SimpleMatrix.identity(6).scale(0.001),
                SimpleMatrix.identity(2).scale(Math.pow(STDEV, 2)), updateA(),
                new SimpleMatrix(new double[][] { { 0, 0 }, { 0, 0 }, { 1, 0 }, { 0, 0 }, { 0, 0 }, { 0, 1 } }),
                new SimpleMatrix(new double[][] { { 1, 0, 0, 0, 0, 0 }, { 0, 0, 0, 1, 0, 0, 0 } }));

        timer.start();

    }

    @Override
    public void execute() {

        t = timer.get();
        timer.reset();

        filter.setA(updateA());

        power = SmartDashboard.getNumber("Power", 0);
        theta = Rotation2d.fromDegrees(SmartDashboard.getNumber("Theta (Degs)", -45));
        u.set(0, 0, (power * theta.getCos()));
        u.set(0, 1, (power * theta.getSin()));
        
        // getting real position of robot

        prevPosReal = new SimpleMatrix(posReal);
        posReal.set(0, 0, field.getRobotPose().getTranslation().getX());
        posReal.set(1, 0, field.getRobotPose().getTranslation().getY());

        // adding Gaussian noise factor

        posNoisy.set(0, 0, (STDEV * rand.nextGaussian()) + posReal.get(0, 0));
        posNoisy.set(1, 0, (STDEV * rand.nextGaussian()) + posReal.get(1, 0));

        filter.runFilter(u, posNoisy);

        // adding numbers to SmartDashboard

        SmartDashboard.putNumber("x (real)", posReal.get(0, 0));
        SmartDashboard.putNumber("y (real)", posReal.get(1, 0));
        SmartDashboard.putNumber("x (noisy)", posNoisy.get(0, 0));
        SmartDashboard.putNumber("y (noisy)", posNoisy.get(1, 0));
        SmartDashboard.putNumber("x (filt.)", filter.getState().get(0, 0));
        SmartDashboard.putNumber("y (filt.)", filter.getState().get(3, 0));
        SmartDashboard.putNumber("vx (est.)", value)
        SmartDashboard.putNumber("vx (filt.)", filter.getState().get(1, 0));
        SmartDashboard.putNumber("vy (filt.)", filter.getState().get(4, 0));
        SmartDashboard.putNumber("ax (input)", u.get(0, 0));
        SmartDashboard.putNumber("ay (input)", u.get(1, 0));
        SmartDashboard.putNumber("ax (filt.)", filter.getState().get(2, 0));
        SmartDashboard.putNumber("ay (filt.)", filter.getState().get(5, 0));

    }

    @Override
    public void end(boolean interrupted) {

        timer.stop();

    }

    /**
     * Updates pose of the robot.
     */

    private void updatePose() {

        field.setRobotPose(pos.get(0, 0), pos.get(1, 0), theta);

    }

    /**
     * Updates A matrix for specific time.
     * 
     * @return Updated A matrix.
     */

    private SimpleMatrix updateA() {

        return new SimpleMatrix(
                new double[][] { { 1, t, Math.pow(t, 2) / 2, 0, 0, 0 }, { 0, 1, t, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 },
                        { 0, 0, 0, 1, t, Math.pow(t, 2) / 2 }, { 0, 0, 0, 0, 1, t }, { 0, 0, 0, 0, 0, 0 } });

    }
    