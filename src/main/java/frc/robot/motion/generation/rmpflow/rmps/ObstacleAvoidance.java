// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motion.generation.rmpflow.rmps;

import org.ejml.simple.SimpleMatrix;

import frc.robot.mapping.Obstacle;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Path;
import frc.robot.mapping.Point;
import frc.robot.mapping.Polygon;
import frc.robot.motion.generation.rmpflow.RMPNode;

/**
 * ObstacleNode works with field obstacles.
 */
public class ObstacleAvoidance extends RMPNode {
    private final double radius;
    private double epsilon = 1e-1;
    private double alpha = 1e-5;
    private double eta = 0;

    /**
     * An Obstacle avoidance RMP that places {@link CollisionAvoidance} RMPs at all
     * polygon vertices and regularly along the sides of the polygon with a distance
     * radius.
     * @param name
     * @param map
     * @param parent
     * @param radius
     */
    public ObstacleAvoidance(String name, ObstacleMap map, RMPNode parent, double radius) {
        super(name, parent);
        addObstacleChildren(map);
        this.radius = radius;
    }

    public ObstacleAvoidance(String name, ObstacleMap map, RMPNode parent, double radius, double epsilon, double alpha, double eta) {
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
        linkChild(new CollisionAvoidance("Child Obstacle Node", this,
                new SimpleMatrix(new double[][] { { point.getX(), point.getY() } }), radius, epsilon, alpha, eta));
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

        if (distance(point1, point2) <= radius) {

            linkChildFromPoint(midpoint);

        } else {

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

}
