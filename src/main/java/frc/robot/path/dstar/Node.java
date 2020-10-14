package frc.robot.path.dstar;

import java.util.Set;
import java.util.LinkedHashSet;
import java.util.Comparator;
import java.util.Collections;
import java.util.Collection;

/** A node along a possible path. */
public class Node extends Point implements Comparable<Node> {
  public Set<Node> edges;
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
  public Node(double x, double y, Obstacle obstacle, Collection<Node> edges) {
    super(x, y);
    this.edges = new LinkedHashSet<Node>(edges);
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
    this(x, y, obstacle, new LinkedHashSet<Node>());
  }
  
  /**
   * Calculates the distance from this node to another node.
   * 
   * @param dest  The node to calculate the distance to.
   * @return  The distance between this node and the specified node.
   */
  public double weightTo(Node dest) {
    return distance(dest);
  }

  /** Returns the next node which greedily minimizes the distance to the goal. */
  public Node getNext() {
    return Collections.min(edges, new Node.GoalDistOrder(this));
  }

  /**
   * Determines if this node is an endpoint of its parent obstacle.
   * 
   * <p>When a single vertex is in question, this method is faster than comparing
   * the vertex against the nodes returned form {@link Obstacle#endpoints(Node)}.
   * 
   * @param source  The node this obstacle is being viewed from.
   * @return  Either true, if this node is an endpoint of its parent obstacle as
   *  seen from the source node, or false, otherwise.
   */
  public boolean isEndpoint(Node source) {
    int idx = obstacle.vertexes.indexOf(this);
    int numVerts = obstacle.vertexes.size();
    Node next = obstacle.vertexes.get((idx + 1) % numVerts);
    Node prev = obstacle.vertexes.get((idx + numVerts - 1) % numVerts);
    double theta_next = getAngle(this, source, next);
    double theta_prev = getAngle(this, source, prev);
    if(theta_next * theta_prev < 0) return false;
    else if(theta_next * theta_prev > 0) return true;
    else if(theta_next == 0 && source.distance(next) < source.distance(this)) return false;
    else if(theta_prev == 0 && source.distance(prev) < source.distance(this)) return false;
    else return true;
  }

  /** Calculates the key of this node used in the D* Lite algorithm.
   * 
   * @return  The minimum of the g and rhs values of this node.
   */
  public double key() {
    return Math.min(g, rhs);
  }

  /** Compares this node to another by D* Lite key value. */
  @Override
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