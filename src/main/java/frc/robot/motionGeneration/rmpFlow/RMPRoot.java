package frc.robot.motionGeneration.rmpFlow;

import org.ejml.simple.SimpleMatrix;

public class RMPRoot extends RMPNode{
	public RMPRoot(String name)
	{
		super(name, null, null, null, null);
	}
	
	public void setRootState(SimpleMatrix x, SimpleMatrix x_dot)
	{
		if(x.numRows() == 1)
			this.x = x.transpose();
		else
			this.x = x;
		if(x_dot.numRows() == 1)
			this.x_dot = x_dot.transpose();
		else
			this.x_dot = x_dot;
	}
	
	@Override
	public void pushforward()
	{
		for(int i = 0; i < getChildren().size(); i++)
			getChildren().get(i).pushforward();
	}
	
	public SimpleMatrix resolve()
	{
		a = m.pseudoInverse().mult(f);
		return a;
	}
	
	public SimpleMatrix solve(SimpleMatrix x, SimpleMatrix x_dot)
	{
		setRootState(x, x_dot);
		pushforward();
		pullback();
		return resolve();
	}
}
