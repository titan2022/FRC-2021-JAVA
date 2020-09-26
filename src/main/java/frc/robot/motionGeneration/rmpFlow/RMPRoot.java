package frc.robot.motionGeneration.rmpFlow;

import org.ejml.simple.SimpleMatrix;

/**
 * The root node of an RMP tree.
 */
public class RMPRoot extends RMPNode{
	/**
	 * A root node of an RMP tree.
	 * @param name of tree
	 */
	public RMPRoot(String name)
	{
		super(name, null);
	}
	
	/**
	 * Updates the states of the tree
	 * @param x The position
	 * @param x_dot The velocity
	 */
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
	
	/**
	 * 	The operator to forward propagate the state from a parent node to its child nodes.
	 *  I.e. Transform an RMP from the domain of a task map to its co-domain.
	 * 	Equation 12 in Riemannian Motion Policies.
	 */
	@Override
	public void pushforward()
	{
		for(int i = 0; i < getChildren().size(); i++)
			getChildren().get(i).pushforward();
	}
	
	/**
	 * Maps an RMP from its natural form to its canonical form.
	 * @return A matrix containing the canonical form.
	 */
	public SimpleMatrix resolve()
	{
		a = m.pseudoInverse().mult(f);
		return a;
	}
	
	/**
	 * Updates the state of the tree and solves for the desired output.
	 * @param x The position.
	 * @param x_dot The velocity.
	 * @return An matrix representing the resolved state of the RMP tree.
	 */
	public SimpleMatrix solve(SimpleMatrix x, SimpleMatrix x_dot)
	{
		setRootState(x, x_dot);
		pushforward();
		pullback();
		return resolve();
	}
}
