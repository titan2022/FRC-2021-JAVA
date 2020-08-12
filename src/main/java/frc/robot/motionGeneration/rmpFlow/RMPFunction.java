package frc.robot.motionGeneration.rmpFlow;

public interface RMPFunction<T> {
	T of(T x, T x_dot);
}
