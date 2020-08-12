package frc.robot.motionGeneration.rmpFlow;

import org.ejml.simple.SimpleMatrix;

public abstract class RMPLeaf extends RMPNode{
	private RMPFunction<SimpleMatrix> RMPf, RMPm;
	
	public RMPLeaf(String name, RMPNode parent
			, EvaluatableFunction<SimpleMatrix> psi
			, EvaluatableFunction<SimpleMatrix> j
			, BiEvaluatableFunction<SimpleMatrix> j_dot)
	{
		super(name, parent, psi, j, j_dot);
	}
	
	public final void evaluate()
	{	
		solveM();
		solveF();
		//f = RMPf.of(x, x_dot);
		//m = RMPm.of(x, x_dot);
		//self.f, self.M = self.RMP_func(self.x, self.x_dot)//This function returns f and m
	}
	
	@Override
	public final void pullback()
	{
		evaluate();
	}
	
	public final void update()
	{
		super.pushforward();
	}
	
	public static SimpleMatrix eye(int size)
	{
		SimpleMatrix eye = new SimpleMatrix(size, size);
		for(int i = 0; i < size; i++)
		{
			eye.set(i , i, 1);
		}
		return eye;
	}
	protected abstract void solveF();
	
	protected abstract void solveM();
}
