package frc.robot.motionGeneration.rmpFlow;

import org.ejml.simple.SimpleMatrix;

public class RMP {
	protected SimpleMatrix x, x_dot, f, m;
	protected EvaluatableFunction<SimpleMatrix> psi, j;
	protected BiEvaluatableFunction<SimpleMatrix> j_dot;
}
