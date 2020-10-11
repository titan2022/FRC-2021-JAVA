package frc.robot.path.dstar;

import java.util.List;

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
  public Obstacle(List<Node> vertexes) {
    this.vertexes = List.copyOf(vertexes);
  }
  
  private static double getAngle(Node base, Node vertex, Node leg) {
    return (Math.atan2(leg.y - vertex.y, leg.x - vertex.x) -
      Math.atan2(base.y - vertex.y, base.x - vertex.x)) % Math.PI;
  }
  
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

  public boolean isEndpoint(int idx, Node source) {
    Node endpoint = vertexes.get(idx);
    Node next = vertexes.get((idx+1) % vertexes.size());
    Node prev = vertexes.get((idx + vertexes.size() -1) % vertexes.size());
    double theta_next = getAngle(endpoint, source, next);
    double theta_prev = getAngle(endpoint, source, prev);
    if(theta_next * theta_prev < 0) return false;
    else if(theta_next * theta_prev > 0) return true;
    else if(theta_next == 0 && source.weightTo(next) < source.weightTo(endpoint)) return false;
    else if(theta_prev == 0 && source.weightTo(prev) < source.weightTo(endpoint)) return false;
    else return true;
  }
}