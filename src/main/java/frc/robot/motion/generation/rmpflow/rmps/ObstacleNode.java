// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motion.generation.rmpflow.rmps;

import org.ejml.simple.SimpleMatrix;

import frc.robot.mapping.Obstacle;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Path;
import frc.robot.mapping.Point;
import frc.robot.motion.generation.rmpflow.RMPNode;

/**
 * ObstacleNode works with field obstacles.
 */
public class ObstacleNode extends RMPNode {
    private double radius;
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
    public ObstacleNode(String name, ObstacleMap map, RMPNode parent, double radius) {
        super(name, parent);
        addObstacleChildren(map);
        this.radius = radius;
    }

    private void addObstacleChildren(ObstacleMap map) {
        Iterable<Obstacle> obstacles = map.getObstacles();
        Path path;
        int nodeCount;
        int i;

        for (Obstacle obstacle : obstacles) {
            path = obstacle.getBoundary();
            nodeCount = (int) (path.getLength() / radius);

            for (i = 0; i < nodeCount; i++) {

                linkChildFromPoint(path.getPos(i * radius));

            }

            linkChildFromPoint(path.getEnd());
        }
    }

    private void linkChildFromPoint(Point point) {
        linkChild(new CollisionAvoidance("Child Obstacle Node", this,
                new SimpleMatrix(new double[][] { { point.getX(), point.getY() } }), radius, epsilon, alpha, eta));
    }
}
