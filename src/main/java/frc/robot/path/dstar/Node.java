package frc.robot.path.dstar;

import java.util.List;
import java.util.ArrayList;

public class Node {
  public List<Node> edges;
  double x;
  double y;
  public double g;
  public double rhs;
  
  public Node(double x, double y, List<Node> edges) {
    this.x = x;
    this.y = y;
    this.edges = edges;
  }
  public Node(double x, double y) {
    this(x, y, new ArrayList<Node>());
  }
  
  public double weightTo(Node dest) {
    return Math.sqrt(Math.pow(x - dest.x, 2) + Math.pow(y - dest.y, 2));
  }

  public double key() {
    return Math.min(g, rhs);
  }
}