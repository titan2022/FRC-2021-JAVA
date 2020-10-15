package frc.robot.path.dstar;

import java.util.List;
import java.util.ArrayList;

/**
 * A polygonal obstacle for path planning.
 * 
 * <p>This class implements a general convex polygonal obstacle with a node at
 * each vertex. Concave obstacles may be represented as multiple convex
 * obstacles, and obstacles with smooth sides may be approximated by a polygon
 * entirely containing the boundaries of the obstacle. Subclasses of this class
 * may provide more efficient implementations for specific types of obstacles.
 */
public class Obstacle {
  public final List<Node> vertexes;
  
  /**
   * Creates an obstacle from a list of its vertexes.
   * 
   * @param vertexes  A list of the vertexes of this obstacle.
   */
  public Obstacle(Iterable<Point> verts) {
    Node vertex, prev, next;
    List<Node> nodes = new ArrayList<Node>();
    for(Point point : verts){
      vertex = new Node(point.x, point.y);
      nodes.add(vertex);
      prev = nodes.get((nodes.size()*2-2) % nodes.size());
      next = nodes.get(0);
      prev.next = next.prev = vertex;
      vertex.next = next;
      vertex.prev = prev;
    }
    vertexes = List.copyOf(nodes);
  }
  
  /**
   * Computes the angle between three nodes.
   * 
   * @param base  The node defining the ray the angle is measured from.
   * @param vertex  The node defining the common endpoint of the rays
   *  forming the angle to measure.
   * @param leg  The node defining the ray the angle is measured to.
   * @return  The signed angle from the ray from {@code vertex} to {@code base}
   *  to the ray from {@code vertex} to {@code leg}. This value is the measure
   *  of that angle in radians, and is always on the interval [-pi, pi].
   */
  private static double getAngle(Node base, Node vertex, Node leg) {
    return (Math.atan2(leg.y - vertex.y, leg.x - vertex.x) -
      Math.atan2(base.y - vertex.y, base.x - vertex.x)) % Math.PI;
  }
  
  /**
   * Determines if a straight line between two nodes pass through this obstacle.
   * 
   * @param a  The node at one endpoint of the straight line.
   * @param b  The node at the other endpoint of the straight line.
   * @return  Either true if the straight-line path from {@code a} to {@code b}
   *  does not pass through the interior of this obstacle, or false is that path
   *  does pass through the interior of this obstacle. If the path includes a
   *  portion of the perimeter of this obstacle, but never crosses the
   *  perimeter, the path is considered not to pass through the interior of this
   *  obstacle, and this method returns true.
   */
  public boolean isClear(Node a, Node b) {
    Node x, y;
    for(int i=0; i<vertexes.size(); i++){
      x = vertexes.get(i);
      y = vertexes.get((i+1) % vertexes.size());
      if(getAngle(a, b, x) * getAngle(a, b, y) < 0 &&
         getAngle(x, y, a) * getAngle(x, y, b) < 0)
        return false;
    }
    return true;
  }

  /**
   * Determines the first and last vertexes of this obstacle visible form a node.
   * 
   * <p>Ignoring the presence of other obstacles, these vertexes, the endpoints
   * of this obstacle as seen from the given node, would be the leftmost and
   * rightmost points of this obstacle visible from the given node. 
   * 
   * @param source  The node to find the endpoints from.
   * @return  The endpoints of this node as seen from the given source node.
   */
  public Node[] endpoints(Node source) {
    Node argmin = null, argmax = null;
    double theta;
    double min = 360.;
    double max = 360.;
    for(Node vertex : vertexes){
      theta = getAngle(vertexes.get(0), source, vertex);
      if(theta > max){
        max = theta;
        argmax = vertex;
      }
      if(theta < min){
        min = theta;
        argmin = vertex;
      }
    }
    return new Node[] {argmin, argmax};
  }
}