package frc.robot.kinematics;

import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;

/**
 * 
 */
public class DifferentialDriveKinematics {
    private double wR, a;
    private MatBuilder DifferentialJacobianBuilder;
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
        QBuilder = new VecBuilder<>(Nat.N2());
        PBuilder = new VecBuilder<>(Nat.N3());
    }

    /**
     * 
     * @param vL
     * @param vR
     * @param phi
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
     * @param vX
     * @param vY
     * @param phi
     * @return
     */
    public Matrix getWheelVelocity(double vX, double vY, double phi)
    {
        return DifferentialJacobian.times(Q);
    }
}