package frc.robot;

import edu.wpi.first.wpiutil.math.*;
import java.lang.Math.*;
public class SwerveDriveKinematics{
    // public Kinematics(){
    // }
    double wR, a;
    MatBuilder SwerveJacobianBuilder;
    Matrix SwerveJacobian;
    VecBuilder QBuilder;
    Matrix Q;
    VecBuilder PBuilder;
    Matrix P;
    public SwerveDriveKinematics(double wheelRadius, double robotWidth){
        wR = wheelRadius;
        a = robotWidth/2.0;

        SwerveJacobianBuilder = new MatBuilder<>(Nat.N3(), Nat.N2());
        QBuilder = new VecBuilder<>(Nat.N2());
        PBuilder = new VecBuilder<>(Nat.N3());

    }

    //heading should be taken from the gyro
    @SuppressWarnings("rawtypes")
    public Matrix getAbsoluteVelocity(double vL, double vR, double phi){
        Q = QBuilder.fill(vR, vL);
        SwerveJacobian = SwerveJacobianBuilder.fill((wR/2)*Math.cos(phi),(wR/2)*Math.cos(phi),(wR/2)*Math.sin(phi),(wR/2)*Math.sin(phi),wR/(2*a),-wR/(2*a));
        return SwerveJacobian.times(Q);
    }
}