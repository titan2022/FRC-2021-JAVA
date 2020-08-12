package frc.robot.motionGeneration.rmpFlow;

import org.ejml.simple.SimpleMatrix;

public class GoalAttractor extends RMPLeaf {
	private double w_u, w_l, sigma, alpha, eta, gain , tolerance;
	private SimpleMatrix goal;
	
	public GoalAttractor(String name, RMPNode parent, SimpleMatrix goal, double w_u, double w_l, double sigma
			, double alpha, double eta, double gain, double tolerance)
	{
		super(name, parent, null, null, null);
		this.w_u = w_u;
		this.w_l = w_l;
		this.sigma = sigma;
		this.alpha = alpha;
		this.eta = eta;
		this.gain = gain;
		this.tolerance = tolerance;
		
		if(goal.numRows() == 1)
			this.goal = goal.transpose();
		else
			this.goal = goal;
		
		updateGoal(this.goal);//Set psi, j, and j_dot
	}
	
	public void solveF()
	{
		double x_norm = x.normF();
		
		double beta = Math.exp(- Math.pow(x_norm, 2) / 2 / Math.pow(sigma, 2));
		double w = (w_u - w_l) * beta + w_l;
		double s = (1 - Math.exp(-2 * alpha * x_norm)) / (1 + Math.exp(-2 * alpha * x_norm));
		
		//SimpleMatrix G = eye(this.goal.numCols() * this.goal.numRows()).scale(w);
		SimpleMatrix grad_Phi;
		if(x_norm > tolerance)
			grad_Phi = x.scale(s / x_norm * w * gain);
		else
			grad_Phi = new SimpleMatrix(x.numRows() , 1);
		SimpleMatrix bx_dot = x_dot.scale(eta * w);
		SimpleMatrix grad_w = x.scale(-beta * (w_u - w_l) / Math.pow(sigma, 2));
		
		double x_dot_norm = x_dot.normF();
		SimpleMatrix xi = grad_w.scale(Math.pow(x_dot_norm, 2)).minus(x_dot.mult(x_dot.transpose()).mult(grad_w).scale(2)).scale(-.5);
		
		this.f = grad_Phi.scale(-1).minus(bx_dot).minus(xi);
	}
	
	public void solveM()
	{
		double x_norm = x.normF();
		
		double beta = Math.exp(- Math.pow(x_norm, 2) / 2 / Math.pow(sigma, 2));
		double w = (w_u - w_l) * beta + w_l;
		//double s = (1 - Math.exp(-2 * alpha * x_norm)) / (1 + Math.exp(-2 * alpha * x_norm));
		
		SimpleMatrix G = eye(this.goal.numCols() * this.goal.numRows()).scale(w);
		
		/*
		SimpleMatrix grad_Phi;
		if(x_norm > tolerance)
			grad_Phi = x.scale(s / x_norm * w * gain);
		else
			grad_Phi = new SimpleMatrix(1 , 1);
		SimpleMatrix bx_dot = x_dot.scale(eta * w);//eta * w * x_dot.get(0 , 0);
		SimpleMatrix grad_w = x.scale(-beta * (w_u - w_l) / Math.pow(sigma, 2));
		
		double x_dot_norm = x_dot.normF();
		SimpleMatrix xi = grad_w.scale(Math.pow(x_dot_norm, 2)).minus(x_dot.mult(x_dot.transpose()).mult(grad_w).scale(2)).scale(-.5);
		*/
		
		this.m = G;
	}
	
	public void updateGoal(SimpleMatrix goal)
	{
		if(goal.numRows() == 1)
			goal = goal.transpose();
		
		int n = goal.numCols() * goal.numRows();
		final SimpleMatrix finalGoal = goal;
		psi = (y) -> y.minus(finalGoal);
		j = (y) -> eye(n);
		j_dot = (SimpleMatrix y, SimpleMatrix y_dot) -> new SimpleMatrix(n, n);
	}
}
