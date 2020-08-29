package frc.robot.motiongeneration.rmpflow;

public interface RMPFunction<T> {
	T of(T x, T x_dot);
}
