package frc.robot.path.dstar;

import java.util.List;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Collections;

public class Node implements Comparable<Node> {
  public List<Node> edges;
  final double x;
  final double y;
  public double g;
  public double rhs;
  public final Obstacle obstacle;
  public boolean visited = false;
  
  public Node(double x, double y, Obstacle obstacle, List<Node> edges) {
    this.x = x;
    this.y = y;
    this.edges = edges;
    this.obstacle = obstacle;
  }
  public Node(double x, double y, Obstacle obstacle) {
    this(x, y, obstacle, new ArrayList<Node>());
  }
  
  public double weightTo(Node dest) {
    return Math.sqrt(Math.pow(x - dest.x, 2) + Math.pow(y - dest.y, 2));
  }

  public Node getNext() {
    return Collections.min(edges, new Node.GoalDistOrder(this));
  }

  public double key() {
    return Math.min(g, rhs);
  }

  public int compareTo(Node o) {
    return (int) Math.signum(o.key() - key());
  }

  private class GoalDistOrder implements Comparator<Node> {
    Node from;
    GoalDistOrder(Node from) {
      this.from = from;
    }
    
    @Override
    public int compare(Node o1, Node o2) {
      return (int) Math.signum(from.weightTo(o1) + o1.g - from.weightTo(o2) - o2.g);
    }
  }
}