package frc.robot.motiongeneration.rmpflow;

import org.ejml.simple.SimpleMatrix;

public class CollisionAvoidance extends RMPLeaf {
	private double r, alpha, eta, epsilon;
	private SimpleMatrix center;
	
	public CollisionAvoidance(String name, RMPNode parent, SimpleMatrix center, double r, double epsilon, double alpha, double eta)
	{
		super(name, parent);
		this.r = r;
		this.alpha = alpha;
		this.eta = eta;
		this.epsilon = epsilon;
		
		if(center.numRows() == 1)
			this.center = center.transpose();
	}

	public SimpleMatrix phi(SimpleMatrix q)
	{
		return new SimpleMatrix(1, 1, false, new double[] {q.minus(center).normF() / r - 1}).transpose();
	}

	public SimpleMatrix j(SimpleMatrix q)
	{
		double scale = 1/q.minus(center).normF();
		return q.minus(center).transpose().divide(r).scale(scale);
	};

	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot)
	{
		SimpleMatrix a = q.minus(center).mult(q.minus(center).transpose())
						  	  .scale(-1 / Math.pow(q.minus(center).normF(), 3));
		SimpleMatrix b = a.plus(eye(center.getNumElements()).scale(1 / q.minus(center).normF()));
		return q_dot.transpose().mult(b).divide(this.r);
	};
	
	public SimpleMatrix solveF()
	{
		double w;
		double grad_w;
		if(x.get(0, 0) < 0)
		{
			w = 1e10;
			grad_w = 0;
		}
		else
		{
			w = 1 / Math.pow(x.get(0, 0), 4);
			grad_w = -4 / Math.pow(x.get(0, 0), 5);
		}
		
		double u = epsilon + Math.min(0, x_dot.get(0, 0)) * x_dot.get(0, 0);
		double g = w * u;
		
		double grad_Phi = alpha * w * grad_w;
		double xi = .5 * Math.pow(x_dot.get(0, 0), 2) * u * grad_w;
		
		double bx_dot = eta * g * x_dot.get(0, 0);
		
		double f_double = -grad_Phi - xi - bx_dot;
		return new SimpleMatrix(1, 1, false, new double[] {Math.min(Math.max(-1e10, f_double), 1e10)});

	}
	
	public SimpleMatrix solveM()
	{
		double w;
		if(x.get(0, 0) < 0)
		{
			w = 1e10;
		}
		else
		{
			w = 1 / Math.pow(x.get(0, 0), 4);
		}
		
		double u = epsilon + Math.min(0, x_dot.get(0, 0)) * x_dot.get(0, 0);
		double g = w * u;
		
		double grad_u = 2 * Math.min(0, x_dot.get(0, 0));
		
		double m_double = g + .5 * x_dot.get(0, 0) * w * grad_u;
		return new SimpleMatrix(1, 1, false, new double[] {Math.min(Math.max(-1e5, m_double), 1e5)});
	}
	
	public double getRadius()
	{
		return r;
	}
	public SimpleMatrix getCenter()
	{
		return center;
	}
}