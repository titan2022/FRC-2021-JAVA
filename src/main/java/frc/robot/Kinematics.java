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

    //method requires angular velocity of both wheels and the heading of the robot.
    //heading should be 
    @SuppressWarnings("rawtypes")
    public Matrix getAbsoluteVelocity(double vL, double vR, double phi){
        Q = QBuilder.fill(vR, vL);
        DifferentialJacobian = DifferentialJacobianBuilder.fill((wR/2)*Math.cos(phi),(wR/2)*Math.cos(phi),(wR/2)*Math.sin(phi),(wR/2)*Math.sin(phi),wR/(2*a),-wR/(2*a));
        return DifferentialJacobian.times(Q);
    }
}