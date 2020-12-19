package frc.robot.motion.generation.rmpflow;

import org.ejml.simple.SimpleMatrix;

public class Damper extends RMPLeaf{
    private double w;
    private double eta;

    public Damper(String name, RMPNode parent, double eta, double w){
        super(name, parent);
        this.eta = eta;
        this.w = w;
    }
         
    public SimpleMatrix phi(SimpleMatrix q)
	{
		return q;
	}

	public SimpleMatrix j(SimpleMatrix q)
	{
		return eye(2);
	};

	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot)
	{
		return new SimpleMatrix(2, 2);
	};


    public SimpleMatrix solveF(){
        SimpleMatrix bx_dot = x_dot.scale(-(eta * w));
        return bx_dot;
    }

    
    public SimpleMatrix solveM(){
        return new SimpleMatrix(1, 1, false, new double[]{w});
    }

}
