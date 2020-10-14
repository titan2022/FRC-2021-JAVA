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

  /**
   * Computes the shortest to the goal and returns the next node along the path.
   * 
   * This method also automatically creates edges to connect nodes to their
   * relevant neighbors as necessary, so connections between nodes do not have
   * to be pre-specified.
   * 
   * @return  The next node along the shortest path to the goal node.
   */
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
        if(v.isEndpoint(v) && isClear(v, endpoint)){
          v.edges.add(endpoint);
          endpoint.edges.add(v);
          updateVertex(endpoint);
        }
      }
    }
    updateVertex(v);
    v.visited = true;
  }

  /**
   * Determines if a straight path between two nodes is free of obstacles.
   * 
   * @param a  The node at one endpoint of the straight path.
   * @param b  The node at the other endpoint of the straight path.
   * @return  Either true, if the path does not pass through the interior of any
   *  obstacles, or false, otherwise.
   * @see Obstacle#isClear(Node, Node)
   */
  public boolean isClear(Node a, Node b) {
    for(Obstacle obs : map)
      if(!obs.isClear(a, b))
        return false;
    return true;
  }

  /**
   * Adds an obstacle to the internal map used to compute the shortest path.
   * 
   * This method automatically connects each vertex of the new obstacle to all
   * relevant neighbors.
   * 
   * This can be used along with {@link #deleteObstacle(Obstacle)} to change an
   * obstacle.
   * 
   * @param obstacle  The obstacle to add to the map.
   */
  public void addObstacle(Obstacle obstacle) {
    map.add(obstacle);
    for(Node node : obstacle.vertexes){
      connect(node);
    }
  }

  /**
   * Removes an obstacle from the internal map used to compute the shortest path.
   * 
   * This method automatically informs the neighbors of each of the vertexes of
   * the removed obstacle and severs the relevant edges.
   * 
   * This can be used along with {@link #addObstacle(Obstacle)} to change an
   * obstacle.
   * 
   * @param obstacle  The obstacle to remove from the map.
   */
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