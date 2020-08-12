package frc.robot.motionGeneration.rmpFlow;

public interface BiEvaluatableFunction<T> {
	T of(T x, T x_dot);
}
