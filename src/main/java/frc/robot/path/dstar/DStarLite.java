package frc.robot.path.dstar;

import java.util.PriorityQueue;
import java.util.Set;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.Iterator;

/** An implementation of the D* Lite dynamic path planning algorithm. */
public class DStarLite {
  private Node start;
  private Node goal;
  private PriorityQueue<Node> queue;
  private Set<Obstacle> map;
  
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
    map = new LinkedHashSet<Obstacle>(Arrays.asList(obstacles));
    queue = new PriorityQueue<Node>();
    queue.add(goal);
  }

  /**
   * Sets the start node for the algorithm.
   * 
   * @param start  The new start node to use.
   */
  public void setStart(Node start) {
    remove(this.start);
    this.start = start;
    connect(start);
  }

  /**
   * Sets the goal node for the algorithm.
   * 
   * @param goal  The new goal node to use.
   */
  public void setGoal(Node goal) {
    remove(this.goal);
    this.goal = goal;
    connect(goal);
  }

  public Node getStart() {
    return start;
  }
  
  public Node getGoal() {
    return goal;
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

  /**
   * Finds the shortest to the goal and returns the next segment along the path.
   * 
   * This method automatically creates edges to connect nodes to their relevant
   * neighbors as necessary, so connections between nodes do not have to be
   * pre-specified.
   * 
   * @param radius  The obstacle growth radius to use. That is, the minimum
   *  distance the path should keep away from the nearest edge.
   * @return  The next path {@link Segment} along the shortest path to goal with
   *  the specified obstacle growth radius, or null if the start point is
   *  already within the specified radius around the goal.
   */
  public Segment getSegment(double radius) {
    Node target = getPath();
    if(start.getDistance(goal) <= radius)
      return null;
    else if(start.getDistance(target) <= radius)
      return target.segmentAround(start, target.getNext());
    else
      return target.segmentFrom(start, radius);
  }

  /**
   * Gets an iterable over the path segments from the start node to the goal.
   * 
   * @param radius  The obstacle growth radius to use. That is, the minimum
   *  distance the path should keep away from the nearest edge.
   * @return An iterable of {@link Segment}s describing the path to the
   *  goal. If the most recently returned path segment ends at a Node that
   *  is still a part of the shortest path to goal after the map is
   *  updated, updates to the map stored in this object will be
   *  reflected by the iterator if the {@link #getPath()} method of this
   *  object is called before the next segment is returned from the
   *  iterator.
   */
  /**
   * Gets an iterable over the path segments from the start node to the goal.
   * 
   * @param radius  The obstacle growth radius to use. That is, the minimum
   *  distance the path should keep away from the nearest edge.
   * @return An iterable of {@link Segment}s describing the shortest path to the
   *  goal. This iterator is automatically updated as the internal map of this
   *  object is updated.
   * @see #getSegment(double)
   * @see #getPath()
   */
  public Iterable<Segment> getSegments(double radius) {
    return new Iterable<Segment>() {
      @Override
      public Iterator<Segment> iterator(){
        return new Iterator<Segment>() {
          private Node curr = start;
          private Point pos = start;

          @Override
          public boolean hasNext() {
            return pos.getDistance(goal) <= radius;
          }

          @Override
          public Segment next() {
            Segment path = curr.nextSegment(pos, radius);
            if(path != null && path.getLength() == 0){
              curr = curr.getNext();
              path = curr.nextSegment(pos, radius);
            }
            pos = path.getEnd();
            return path;
          }
        };
      }
    };
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
        if(v.isEndpoint(endpoint) && isClear(v, endpoint)){
          v.edges.add(endpoint);
          endpoint.edges.add(v);
          updateVertex(endpoint);
        }
      }
    }
    updateVertex(v);
    v.visited = true;
  }

  private void remove(Node v) {
    for(Node neighbor : v.edges){
      neighbor.edges.remove(v);
      updateVertex(neighbor);
    }
    queue.remove(v);
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
    for(Node node : obstacle){
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
    for(Node vertex : obstacle){
      remove(vertex);
    }
  }

  /**
   * Returns an iterator over all endpoints visible from a given node.
   * 
   * <p>See {@link Obstacle#endpoint(Node)} for more information about endpoints.
   * The iterator returned by this method considers the start and goal nodes
   * to be endpoints visible from every location.
   * 
   * @param source  The node to find the endpoints from.
   */
  public Iterable<Node> endpoints(Node source) {
    return new Iterable<Node>() {
      @Override
      public Iterator<Node> iterator() {
        return new Iterator<Node>() {
          private Iterator<Obstacle> mapIter;
          private Node[] ends;
          private int idx=0;

          @Override
          public Node next() {
            if(mapIter == null){
              mapIter = map.iterator();
              if(mapIter.hasNext()) ends = mapIter.next().endpoints(source);
              else ends = new Node[]{};
              return goal;
            }
            while(idx >= ends.length){
              if(!mapIter.hasNext()){
                ends = null;
                return start;
              }
              idx = 0;
              ends = mapIter.next().endpoints(source);
            }
            return ends[idx++];
          }

          @Override
          public boolean hasNext() {
            return ends != null || mapIter == null;
          }
        };
      }
    };
  }
}
