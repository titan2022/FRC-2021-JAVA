package frc.robot.motionGeneration.rmpFlow;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

/**
 * A wrapper class that allows Lambda function to be used with
 */
public class RMP extends RMPLeaf {
    private Function<SimpleMatrix, SimpleMatrix> phi, j;
    private BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> j_dot;
    private Supplier<SimpleMatrix> f, m;

	public RMP(String name, RMPNode parent
				, Function<SimpleMatrix, SimpleMatrix> phi, Function<SimpleMatrix, SimpleMatrix> j
				, BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> j_dot
				, Supplier<SimpleMatrix> f, Supplier<SimpleMatrix> m)
    {
        super(name, parent);
        this.phi = phi;
        this.j = j;
        this.j_dot = j_dot;
    }

    /**
	 * Differentiable task map that relates the configuration space to the task space.
	 * @param q is the configuration space
	 * @return the task space
	 */
	public SimpleMatrix phi(SimpleMatrix q)
	{
		return phi.apply(q);
	}

	/**
	 * Jacobian of the task map phi
	 * @param q is the configuration space
	 * @return
	 */
	public SimpleMatrix j(SimpleMatrix q)
	{
		return j.apply(q);
	}

	/**
	 * Second derivative Jacobian of the task map phi
	 * @param q is the configuration space
	 * @param q_dot is the configuration space
	 * @return differentiated Jacobian of configuration space to task space 
	 */
	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot)
	{
		return j_dot.apply(q, q_dot);//TODO decide if to implement like goal Attractor J_dot
    }
    
    /**
	 * Solves for F
	 * , where F is the motion policy that describes the dynamical system as a second-order differential equation that uses position and velocity.
	 * @return F 
	 */
    protected SimpleMatrix solveF()
    {
        return f.get();
    }
	
	/**
	 * Solves for M
	 * , where M is the canonical version of the Riemannian metric A
	 * @return M
	 */
    protected SimpleMatrix solveM()
    {
        return m.get();
    }
}
