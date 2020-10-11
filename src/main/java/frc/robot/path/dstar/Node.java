package frc.robot.path.dstar;

import java.util.List;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Collections;

/** A node along a possible path. */
public class Node implements Comparable<Node> {
  public List<Node> edges;
  final double x;
  final double y;
  public double g;
  public double rhs;
  public final Obstacle obstacle;
  public boolean visited = false;
  
  /**
   * Creates a node at a specific position with a parent obstacle.
   * 
   * @param x  The x coordinate of this node.
   * @param y  The y coordinate of this node.
   * @param obstacle  The parent obstacle of this node. Should not be null.
   * @param edges  A list of the neighbors of this node.
   */
  public Node(double x, double y, Obstacle obstacle, List<Node> edges) {
    this.x = x;
    this.y = y;
    this.edges = edges;
    this.obstacle = obstacle;
  }
  /**
   * Creates a node at a specific position with a parent obstacle.
   * 
   * <p>This constructor initializes the edges of this node to an empty list.
   * Use {@link #Node(double, double, Obstacle, List<Node>)} to specify the
   * neighbors of this node on creation.
   * 
   * @param x  The x coordinate of this node.
   * @param y  The y coordinate of this node.
   * @param obstacle  The parent obstacle of this node. Should not be null.
   */
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