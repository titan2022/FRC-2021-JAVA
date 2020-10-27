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
  public boolean visited = false;
  public Node next;
  public Node prev;
  
  /**
   * Creates a node at a specific position.
   * 
   * @param x  The x coordinate of this node.
   * @param y  The y coordinate of this node.
   * @param prev  The node to link to as the previous vertex of this node's
   *  parent obstacle. Must not be null.
   * @param next  The node to link to as the next vertex of this node's  parent
   *  obstacle. Must not be null
   * @param edges  The neighbors of this node.
   * @throws IllegalArgumentException  One or both of the values passed as
   *  {@code prev} and {@code next} is null.
   */
  public Node(double x, double y, Node prev, Node next, Collection<Node> edges)
      throws IllegalArgumentException {
    super(x, y);
    this.edges = new LinkedHashSet<Node>(edges);
    this.prev = prev;
    this.next = next;
  }
  /**
   * Creates a node at a specific position.
   * 
   * <p>This constructor initializes the edges of this node to an empty set.
   * Use {@link #Node(double, double, Node, Node, Collection<Node>)} to specify
   * the neighbors of this node on creation.
   * 
   * @param x  The x coordinate of this node.
   * @param y  The y coordinate of this node.
   * @param prev  The node to link to as the previous vertex of this node's
   *  parent obstacle. Must not be null.
   * @param next  The node to link to as the next vertex of this node's  parent
   *  obstacle. Must not be null
   * @throws IllegalArgumentException  One or both of the values passed as
   *  {@code prev} and {@code next} is null.
   */
  public Node(double x, double y, Node prev, Node next)
      throws IllegalArgumentException {
    this(x, y, prev, next, new LinkedHashSet<Node>());
  }
  /**
   * Creates a node at a specific position.
   * 
   * <p>This constructor links this node to a single other node as both the
   * previous and next node in this node's parent obstacle. See
   * {@link #Node(double, double, Node, Node, Collection)} to specify
   * different previous and next links for this node.
   * 
   * @param x  The x coordinate of this node.
   * @param y  The y coordinate of this node.
   * @param link  The node to link to as the previous and next node in this
   *  node's parent obstacle. Must not be null.
   * @param edges  A list of the neighbors of this node.
   * @throws IllegalArgumentException  The value passed as {@code link} is null.
   */
  public Node(double x, double y, Node link, Collection<Node> edges)
      throws IllegalArgumentException {
    this(x, y, link, link, edges);
  }
  /**
   * Creates a node at a specific position.
   * 
   * <p>This constructor links this node to a single other node as both the
   * previous and next node in this node's parent obstacle. It also initializes
   * the edges of this node as an empty set. See
   * {@link #Node(double, double, Node, Node, Collection)},
   * {@link #Node(double, double, Node, Collection)}, and
   * {@link #Node(double, double, Node, Node)} to specify one or both of these
   * parameters.
   * 
   * @param x  The x coordinate of this node.
   * @param y  The y coordinate of this node.
   * @param link  The node to link to as the previous and next node in this
   *  node's parent obstacle. Must not be null.
   * @throws IllegalArgumentException  The value passed as {@code link} is null.
   */
  public Node(double x, double y, Node link) throws IllegalArgumentException {
    this(x, y, link, link);
  }
  /**
   * Creates a node at a specific position.
   * 
   * <p>This constructor links this node to itself as both the previous and
   * next node in this node's parent obstacle, effectively making this node its
   * own obstacle. See {@link #Node(double, double, Node, Node, Collection)} to
   * specify previous and next links for this node.
   * 
   * @param x  The x coordinate of this node.
   * @param y  The y coordinate of this node.
   * @param edges  A list of the neighbors of this node.
   */
  public Node(double x, double y, Collection<Node> edges) {
    super(x, y);
    this.edges = new LinkedHashSet<Node>(edges);
    this.prev = this;
    this.next = this;
  }
  /**
   * Creates a node at a specific position.
   * 
   * <p>This constructor links this node to itself as both the previous and
   * next node in this node's parent obstacle, effectively making this node its
   * own obstacle. This also initializes this node's edges to an empty set. See
   * {@link #Node(double, double, Node, Node, Collection)} for a full list of
   * possible parameters.
   * 
   * @param x  The x coordinate of this node.
   * @param y  The y coordinate of this node.
   */
  public Node(double x, double y) {
    super(x, y);
    this.edges = new LinkedHashSet<Node>();
    this.prev = this;
    this.next = this;
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
    double theta_next = getAngle(this, source, next).getRadians();
    double theta_prev = getAngle(this, source, prev).getRadians();
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
    return (int) Math.signum(key() - o.key());
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