package frc.robot;

import edu.wpi.first.wpiutil.math.*;
import java.lang.Math.*;
public class Kinematics{
    // public Kinematics(){
    // }
    double wR, a;
    MatBuilder DifferentialJacobianBuilder;
    Matrix DifferentialJacobian;
    VecBuilder QBuilder;
    Matrix Q;
    VecBuilder PBuilder;
    Matrix P;
    public Kinematics(double wheelRadius, double robotWidth){
        wR = wheelRadius;
        a = robotWidth/2.0;

        DifferentialJacobianBuilder = new MatBuilder<>(Nat.N3(), Nat.N2());
        QBuilder = new VecBuilder<>(Nat.N2());
        PBuilder = new VecBuilder<>(Nat.N3());

    }

    public Matrix getAbsoluteVelocity(double vL, double vR){
        Q = QBuilder.fill(vL, vR);
        DifferentialJacobian = DifferentialJacobianBuilder.fill();
    }
}