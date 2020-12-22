package frc.robot.motioncontrol.kalmanfilter.auxiliary;

import java.util.Random;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Timer;
import static edu.wpi.first.wpilibj.geometry.Rotation2d.fromDegrees;
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
    private SimpleMatrix u; // input format: [power,theta] ^ T
    private SimpleMatrix pos; // measurement format: [x,y] ^ T

    public KalmanFilterTestCommand() {

        addRequirements();

    }

    @Override
    public void initialize() {

        rand = new Random();
        timer = new Timer();
        field = new Field2d();
        u = new SimpleMatrix(new double[][] { { 25 }, { 45 } });
        pos = new SimpleMatrix(new double[][] { { 1 }, { 1 } });
        SmartDashboard.putNumber("Power", u.get(0, 0));
        SmartDashboard.putNumber("Theta (Degs)", u.get(1, 0));
        setPose();
        timer.start();

    }

    @Override
    public void execute() {

        t = timer.get();
        timer.reset();

    }

    @Override
    public void end(boolean interrupted) {

        timer.stop();

    }

    /**
     * Makes a measurement noisy (Gaussian)
     * 
     * @param actual - Actual field position.
     * @param stdev  - Standard deviation for noise.
     * @return Gaussian noisy measurement.
     */

    private double getNoisy(double actual, double stdev) {

        return (stdev * rand.nextGaussian() + actual);

    }

    /**
     * Sets pose of the robot
     */

    private void setPose() {

        field.setRobotPose(pos.get(0, 0), pos.get(1, 0), fromDegrees(u.get(1, 0)));

    }

    /**
     * Updates A matrix for specific time
     */

    private SimpleMatrix updateA() {

        return new SimpleMatrix(
                new double[][] { { 1, t, Math.pow(t, 2) / 2, 0, 0, 0 }, { 0, 1, t, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 },
                        { 0, 0, 0, 1, t, Math.pow(t, 2) / 2 }, { 0, 0, 0, 0, 1, t }, { 0, 0, 0, 0, 0, 1 } });

    }

}