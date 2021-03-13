package frc.robot.motion.generation.rmpflow.rmps;

import org.ejml.simple.SimpleMatrix;

import frc.robot.motion.generation.rmpflow.RMPNode;

public class Cartesian2d extends RMPNode {
    private final int xDim;
    private final int yDim;

    public Cartesian2d(String name, RMPNode parent, int xDim, int yDim) {
        super(name, parent);
        this.xDim = xDim;
        this.yDim = yDim;
    }

    @Override
    public SimpleMatrix psi(SimpleMatrix x) {
        return new SimpleMatrix(new double[][]{{x.get(xDim)}, {x.get(yDim)}});
    }

    @Override
    public SimpleMatrix j(SimpleMatrix x) {
        SimpleMatrix jacobian = new SimpleMatrix(2, x.getNumElements());
        jacobian.set(0, xDim, 1);
        jacobian.set(1, yDim, 1);
        return jacobian;
    }

    @Override
    public SimpleMatrix j_dot(SimpleMatrix x, SimpleMatrix x_dot) {
        return new SimpleMatrix(2, x.getNumElements());
    }
}
