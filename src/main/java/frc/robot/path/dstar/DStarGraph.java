package frc.robot.path.dstar;

import java.util.HashMap;
import java.util.Map;
import java.util.NavigableMap;
import java.util.PriorityQueue;
import java.util.Queue;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.mapping.Obstacle;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Point;

public class DStarGraph {
    private final Queue<DStarNode> queue = new PriorityQueue<DStarNode>();
    private final DStarNode goal;
    private DStarNode start;
    public final ObstacleMap map;
    public final double radius;
    private Map<Obstacle, NavigableMap<Point, DStarNode>> obstacleSets = new HashMap<>();

    public DStarGraph(ObstacleMap map, Translation2d start, Translation2d goal, double radius) {
        this.map = map;
        this.goal = new DStarNode(goal, queue, 0, 0);
        this.start = new DStarNode(start, queue);
        this.radius = radius;
    }
    public DStarGraph(ObstacleMap map, Translation2d start, Translation2d goal) {
        this(map, start, goal, 0);
    }
    public DStarGraph(ObstacleMap map, Translation2d goal, double radius) {
        this(map, goal, goal, radius);
    }
    public DStarGraph(ObstacleMap map, Translation2d goal) {
        this(map, goal, 0);
    }
}
