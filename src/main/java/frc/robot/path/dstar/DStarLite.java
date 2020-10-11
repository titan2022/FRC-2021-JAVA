package frc.robot.path.dstar;

import java.util.PriorityQueue;
import java.util.List;
import java.util.Arrays;

/** An implementation of the D* Lite dynamic path planning algorithm. */
public class DStarLite {
  private Node start;
  private Node goal;
  private PriorityQueue<Node> queue;
  private List<Obstacle> map;
  
  /**
   * Creates an instance of the D* Lite algorithm.
   * 
   * @param start  The start node for the algorithm.
   *  This will change as the robot moves.
   * @param goal  The goal node for the algorithm.
   * @param obstacles  The obstacles to avoid along the path.
   */
  public DStarLite(Node start, Node goal, Obstacle... obstacles) {
    this.start = start;
    this.goal = goal;
    map = Arrays.asList(obstacles);
    queue = new PriorityQueue<Node>();
    queue.add(goal);
  }

  public Node getPath() {
    while(queue.size() > 0 && (queue.peek().key() < start.key() || start.g != start.rhs)){
      Node v = queue.poll();
      if(v.g > v.rhs){
        v.g = v.rhs;
      }
      else if(v.g < v.rhs){
        v.g = Double.POSITIVE_INFINITY;
        updateVertex(v);
      }
      if(!v.visited) connect(v);
      for(Node e: v.edges){
        updateVertex(e);
      }
    }
    return start.getNext();
  }

  private void updateVertex(Node v) {
    if(v != goal){
      Node next = v.getNext();
      v.rhs = v.weightTo(next) + next.g;
    }
    queue.remove(v);
    if(v.rhs != v.g) queue.add(v);
  }

  private void connect(Node v) {
    for(Obstacle obs : map){
      Node[] endpoints = obs.endpoints(v);
      for(int i=0; i<endpoints.length; i++){
        Node endpoint = endpoints[i];
        if(v.obstacle.isEndpoint(i, v) && isClear(v, endpoint)){
          v.edges.add(endpoint);
          endpoint.edges.add(v);
          updateVertex(endpoint);
        }
      }
    }
    updateVertex(v);
    v.visited = true;
  }

  public boolean isClear(Node a, Node b) {
    for(Obstacle obs : map)
      if(!obs.isClear(a, b))
        return false;
    return true;
  }

  public void addObstacle(Obstacle obstacle) {
    map.add(obstacle);
    for(Node node : obstacle.vertexes){
      connect(node);
    }
  }

  public void deleteObstacle(Obstacle obstacle) {
    map.remove(obstacle);
    for(Node vertex : obstacle.vertexes){
      for(Node neighbor : vertex.edges){
        neighbor.edges.remove(vertex);
        updateVertex(neighbor);
      }
      queue.remove(vertex);
    }
  }
}