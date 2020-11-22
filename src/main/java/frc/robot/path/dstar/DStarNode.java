package frc.robot.path.dstar;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

import edu.wpi.first.wpilibj.geometry.Translation2d;

import frc.robot.mapping.Point;
import frc.robot.mapping.Path;

class DStarNode extends Point {
    private final Queue<DStarNode> queue; 
    private double g = Double.POSITIVE_INFINITY;
    private final Map<DStarNode, Path> edges = new HashMap<>();
    DStarNode next = null;

    DStarNode(Translation2d position, Queue<DStarNode> queue) {
        super(position);
        this.queue = queue;
    }
}
