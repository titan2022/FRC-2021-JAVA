package frc.robot.kinematics;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;

/**
 * 
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
     * @return
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
     * @return
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
     * @param vX
     * @param vY
     * @param dPhi
     * @param phi
     * @return
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