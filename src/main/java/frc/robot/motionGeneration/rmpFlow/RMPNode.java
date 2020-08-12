package frc.robot.motionGeneration.rmpFlow;
import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

public class RMPNode {
	private String name;
	private RMPNode parent;
	private ArrayList<RMPNode> children = new ArrayList<RMPNode>();
	protected EvaluatableFunction<SimpleMatrix> psi, j;
	protected BiEvaluatableFunction<SimpleMatrix> j_dot;
	protected SimpleMatrix x, x_dot, f, a, m;
	
	public RMPNode(String name, RMPNode parent
			, EvaluatableFunction<SimpleMatrix> psi
			, EvaluatableFunction<SimpleMatrix> j
			, BiEvaluatableFunction<SimpleMatrix> j_dot)
	{
		this.name = name;
		this.parent = parent;
		if(parent != null)
			parent.addChild(this);
		this.psi = psi;
		this.j = j;
		this.j_dot = j_dot;
		this.x = null;
		this.x_dot = null;
		this.f = null;
		this.a = null;
		this.m = null;
	}
	
	public void addChild(RMPNode child)
	{
		children.add(child);
	}
	
	public ArrayList<RMPNode> getChildren()
	{
		return children;
	}
	
	public void pushforward()
	{
		if(psi != null && j != null)
		{
			x = psi.of(parent.x);//Pass parent.x to the psi function
			x_dot = j.of(parent.x).mult(parent.x_dot);
			//x_dot = x_dot.scale(j.of(parent.x).dot(parent.x_dot)/getMagnitude(parent.x_dot));//Might throw exception
			//dotProduct(j.of(parent.x), parent.x_dot);//dot product of the jacobian of parent and parent x_dot
		}
		
		for(int i = 0; i < children.size(); i++)
			children.get(i).pushforward();
	}
	
	public void pullback()
	{
		for(int i = 0; i < children.size(); i++)
			children.get(i).pullback();
		
		f = x.copy();//f = np.zeros_like(x, doubles);//return an array of the same size as x with zerores
		f.zero();
		int mSize = Math.max(x.numCols(), x.numRows());
		if(m == null)
			m = new SimpleMatrix(mSize, mSize);
		else
			m.reshape(mSize, mSize);//m = np.zeros((max(self.x.shape), max(self.x.shape)), doubles);
		m.zero();
		
		for(int i = 0; i < children.size(); i++)
		{
			RMPNode child = children.get(i);
			SimpleMatrix J_child = child.j.of(x); //child's Jacobian of x
			SimpleMatrix J_dot_child = child.j_dot.of(x, x_dot);
			
			if(child.f != null && child.name != null)
			{
				//Jdot(f-M*J*dot*x)
				f = f.plus(J_child.transpose().mult(child.f.minus(
								child.m.mult(J_dot_child).mult(x_dot))));
				//f += np.dot(J_child.T , (child.f - np.dot(
				//		np.dot(child.M, J_dot_child), self.x_dot)));
				m = m.plus(J_child.transpose().mult(child.m).mult(J_child));
				//m += np.dot(np.dot(J_child.T, child.M), J_child);
			}
		}
	}
}
