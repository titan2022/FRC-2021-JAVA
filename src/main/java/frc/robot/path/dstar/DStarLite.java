package frc.robot.path.dstar;

import java.util.PriorityQueue;
import java.util.List;
import java.util.Arrays;

public class DStarLite {
  private Node start;
  private Node goal;
  private PriorityQueue<Node> queue;
  private List<Obstacle> map;
  
  public DStarLite(Node start, Node goal, Obstacle... obstacles) {
    this.start = start;
    this.goal = goal;
    map = Arrays.asList(obstacles);
    queue = new PriorityQueue<Node>();
  }
}