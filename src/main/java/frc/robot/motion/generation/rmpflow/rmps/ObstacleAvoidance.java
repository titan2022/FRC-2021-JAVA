package frc.robot.motion.generation.rmpflow.rmps;

import org.ejml.simple.SimpleMatrix;

import frc.robot.mapping.Obstacle;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Point;
import frc.robot.mapping.Polygon;
import frc.robot.motion.generation.rmpflow.RMPNode;
import frc.robot.motion.generation.rmpflow.RMPRoot;

/**
 * ObstacleNode works with field obstacles.
 */
public class ObstacleAvoidance extends RMPNode {
    private final double radius;
    private double epsilon = .2;
    private double alpha = 1e-5;
    private double eta = 2;

    /**
     * An Obstacle avoidance RMP that places {@link CollisionAvoidance} RMPs at all
     * polygon vertices and regularly along the sides of the polygon with a distance
     * radius.
     * 
     * @param name
     * @param map
     * @param parent
     * @param radius
     */
    public ObstacleAvoidance(String name, ObstacleMap map, RMPNode parent, double radius) {
        super(name, parent);
        this.radius = radius;
        addObstacleChildren(map);
    }

    public ObstacleAvoidance(String name, ObstacleMap map, RMPNode parent, double radius, double epsilon, double alpha,
            double eta) {
        this(name, map, parent, radius);
        this.epsilon = epsilon;
        this.alpha = alpha;
        this.eta = eta;
    }

    private void addObstacleChildren(ObstacleMap map) {

        Iterable<Obstacle> obstacles = map.getObstacles();

        for (Obstacle obstacle : obstacles) {

            placePolygonPoints(((Polygon) obstacle).getVertices());

        }
    }

    private void linkChildFromPoint(Point point) {
        linkChild(new CollisionAvoidance("Child of Obstacle Avoidance Node", this,
                new SimpleMatrix(1, 2, false, new double[] { point.getX(), point.getY() }), radius, epsilon, alpha,
                eta));
    }

    private void placePolygonPoints(Point[] vertexArray) {

        for (int i = 0; i < vertexArray.length - 1; i++) {

            linkChildFromPoint(vertexArray[i]);
            placeSegmentPoints(vertexArray[i], vertexArray[i + 1]);

        }

        linkChildFromPoint(vertexArray[vertexArray.length - 1]);
        placeSegmentPoints(vertexArray[vertexArray.length - 1], vertexArray[0]);

    }

    private void placeSegmentPoints(Point point1, Point point2) {

        Point midpoint = midpoint(point1, point2);
        linkChildFromPoint(midpoint);

        if (distance(point1, point2) > radius) {

            placeSegmentPoints(point1, midpoint);
            placeSegmentPoints(midpoint, point2);

        }

    }

    private Point midpoint(Point point1, Point point2) {

        return point1.plus(point2).div(2);

    }

    private double distance(Point point1, Point point2) {

        return point1.minus(point2).getNorm();

    }

    @Override
	public SimpleMatrix psi(SimpleMatrix q)
	{
		return new SimpleMatrix(2, 1, false, new double[] {q.get(0), q.get(1)});
	}

    @Override
	public SimpleMatrix j(SimpleMatrix q)
	{
		return new SimpleMatrix(new double[][]{{1, 0, 0}, {0, 1, 0}}); //TODO: verify if this is the correct j_dot function
	}

    @Override
	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot)
	{
		return new SimpleMatrix(2, 3); //TODO: verify if this is the correct j_dot function
    }
    
    // TEMPORARY
    // TODO: REMOVE TEMPORARY CLASS
    public static class ObstacleAvoidanceStatic {

        public static void addRootObstacleChildren(ObstacleMap map, RMPRoot root, double radius) {

            Iterable<Obstacle> obstacles = map.getObstacles();
    
            for (Obstacle obstacle : obstacles) {
    
                placePolygonPoints(((Polygon) obstacle).getVertices(), root, radius);
    
            }
        }
    
        private static void linkChildFromPoint(Point point, RMPRoot root, double radius) {
            root.linkChild(new CollisionAvoidance("Child of Obstacle Avoidance Node", root,
                    new SimpleMatrix(1, 2, false, new double[] { point.getX(), point.getY() }), radius, 0.2, 1e-5, 2));
        }

        private static void placePolygonPoints(Point[] vertexArray, RMPRoot root, double radius) {

            for (int i = 0; i < vertexArray.length - 1; i++) {
    
                linkChildFromPoint(vertexArray[i], root, radius);
                placeSegmentPoints(vertexArray[i], vertexArray[i + 1], root, radius);
    
            }
    
            linkChildFromPoint(vertexArray[vertexArray.length - 1], root, radius);
            placeSegmentPoints(vertexArray[vertexArray.length - 1], vertexArray[0], root, radius);
    
        }
    
        private static void placeSegmentPoints(Point point1, Point point2, RMPRoot root, double radius) {
    
            Point midpoint = midpoint(point1, point2);
            linkChildFromPoint(midpoint, root, radius);
    
            if (distance(point1, point2) > radius) {
    
                placeSegmentPoints(point1, midpoint, root, radius);
                placeSegmentPoints(midpoint, point2, root, radius);
    
            }
    
        }

        private static Point midpoint(Point point1, Point point2) {

            return point1.plus(point2).div(2);
    
        }
    
        private static double distance(Point point1, Point point2) {
    
            return point1.minus(point2).getNorm();
    
        }

    }

}
