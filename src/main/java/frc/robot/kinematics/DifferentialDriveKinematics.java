package frc.robot.kinematics;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;

/**
 * This class simulates the kinematics of a differential drive system. Using linear algebra, we can find the forward and reverse kinematics given their respective parameters. 
 */
public class DifferentialDriveKinematics {
    private double wR, a;
    private MatBuilder DifferentialJacobianBuilder, InverseDifferentialJacobianBuilder;
    private Matrix DifferentialJacobian;
    private VecBuilder QBuilder, PBuilder;
    private Matrix Q, P;

    /**
     * 
     * @param wheelRadius
     * @param robotWidth
     */
    public DifferentialDriveKinematics(double wheelRadius, double robotWidth){
        wR = wheelRadius;
        a = robotWidth/2.0;

        DifferentialJacobianBuilder = new MatBuilder<>(Nat.N3(), Nat.N2());
        InverseDifferentialJacobianBuilder = new MatBuilder<>(Nat.N2(), Nat.N3());
        QBuilder = new VecBuilder<>(Nat.N2());
        PBuilder = new VecBuilder<>(Nat.N3());
    }

    /**
     * 
     * @param vL - the rotational velocity of the left wheel
     * @param vR - the rotational velocity of the right wheel
     * @param phi - the absolute rotational pose of the robot chassis in radians
     * @return - returns a matrix which contains translational velocity of x and y position as well as new rotation
     */
    public Matrix getAbsoluteVelocity(double vL, double vR, double phi){
        Q = QBuilder.fill(vR, vL);
        DifferentialJacobian = DifferentialJacobianBuilder.fill((wR/2)*Math.cos(phi),(wR/2)*Math.cos(phi)
                                                                ,(wR/2)*Math.sin(phi),(wR/2)*Math.sin(phi)
                                                                ,wR/(2*a)            ,-wR/(2*a));
        return DifferentialJacobian.times(Q);
    }

    /**
     * 
     * @param chassisSpeeds an object that encapsulates vx, vy, and rotational velocity
     * @param phi the absolute heading of the robot
     * @return returns a matrix of the wheel velocities and pose
     */
    public Matrix getWheelVelocity(ChassisSpeeds chassisSpeeds, double phi)
    {
        double vX = chassisSpeeds.vxMetersPerSecond;
        double vY = chassisSpeeds.vyMetersPerSecond;
        double dPhi = chassisSpeeds.omegaRadiansPerSecond;
        P = PBuilder.fill(vX, vY, dPhi);
        DifferentialJacobian = InverseDifferentialJacobianBuilder.fill(Math.cos(phi), Math.sin(phi), a,
                                                                        Math.cos(phi), Math.sin(phi), -a);
        DifferentialJacobian = DifferentialJacobian.times(1.0/wR);
        return DifferentialJacobian.times(P);
    }

    /**
     * 
     * @param vX velocity in x direction
     * @param vY velocity in y direction
     * @param dPhi rotational velocity
     * @param phi initial angle of robot
     * @return returns a matrix of the wheel velocities and pose
     */
    public Matrix getWheelVelocity(double vX, double vY, double dPhi, double phi)
    {
        P = PBuilder.fill(vX, vY, dPhi);
        DifferentialJacobian = InverseDifferentialJacobianBuilder.fill(Math.cos(phi), Math.sin(phi), a,
                                                                        Math.cos(phi), Math.sin(phi), -a);
        DifferentialJacobian = DifferentialJacobian.times(1.0/wR);
        return DifferentialJacobian.times(P);
    }
}
