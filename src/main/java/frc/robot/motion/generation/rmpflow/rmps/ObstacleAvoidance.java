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
import frc.robot.subsystems.DifferentialDriveSubsystem;

/**
 * ObstacleNode works with field obstacles.
 */
public class ObstacleAvoidance extends RMPNode {

    private final static double RADIUS = DifferentialDriveSubsystem.ROBOT_TRACK_WIDTH / 2;
    private final static double epsilon = 1e-1;
    private final static double alpha = 1e-5;
    private final static double eta = 0;

    public ObstacleAvoidance(String name, ObstacleMap map, RMPNode parent) {

        super(name, parent);
        addObstacleChildren(map);

    }

    private void addObstacleChildren(ObstacleMap map) {

        Iterable<Obstacle> obstacles = map.getObstacles();
        Path path;
        int nodeCount;
        int i;

        for (Obstacle obstacle : obstacles) {

            path = obstacle.getBoundary();
            nodeCount = (int) (path.getLength() / RADIUS);

            for (i = 0; i < nodeCount; i++) {

                linkChildFromPoint(path.getPos(i * RADIUS));

            }

            linkChildFromPoint(path.getEnd());

        }

    }

    private void linkChildFromPoint(Point point) {

        linkChild(new CollisionAvoidance("Child Obstacle Node", this,
                new SimpleMatrix(new double[][] { { point.getX(), point.getY() } }), RADIUS, epsilon, alpha, eta));

    }

}
