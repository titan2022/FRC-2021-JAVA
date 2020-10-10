package frc.robot.path.dstar;

import java.util.List;

public class Obstacle {
  public final List<Node> vertexes;
  
  public Obstacle(List<Node> vertexes) {
    this.vertexes = List.copyOf(vertexes);
  }
  
  private static double getAngle(Node base, Node vertex, Node leg) {
    return (Math.atan2(leg.y - vertex.y, leg.x - vertex.x) -
      Math.atan2(base.y - vertex.y, base.x - vertex.x)) % 180.;
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
}