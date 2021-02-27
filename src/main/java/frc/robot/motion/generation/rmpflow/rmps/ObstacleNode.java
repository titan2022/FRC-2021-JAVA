// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motion.generation.rmpflow.rmps;

import frc.robot.mapping.Obstacle;
import frc.robot.mapping.ObstacleMap;
import frc.robot.motion.generation.rmpflow.RMPNode;

/**
 * ObstacleNode works with field obstacles.
 */
public class ObstacleNode extends RMPNode {

    public ObstacleNode(ObstacleMap map, RMPNode parent) {

        super("Obstacle Node", parent);

    }

    private addObstacleChildren(ObstacleMap map) {

        Iterable<Obstacle> obstacles = map.getObstacles();      

        for (Obstacle obstacle : obstacles) {

            

        }

    }

}
