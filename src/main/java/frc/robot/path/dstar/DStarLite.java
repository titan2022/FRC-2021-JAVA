package frc.robot.path.dstar;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.mapping.CompoundPath;
import frc.robot.mapping.LinearSegment;
import frc.robot.mapping.Obstacle;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Path;
import frc.robot.mapping.Point;

/** A D* Lite graph and path planning algorithm. */
public class DStarLite {
    private final Queue<Node> queue = new PriorityQueue<Node>();
    private final Node goal;
    private Node start;
    public final ObstacleMap map;
    public final double radius;
    private Map<Obstacle, NavigableMap<Point, Node>> obstacleSets = new HashMap<>();

    /**
     * Creates a DStarLite path planner.
     * 
     * @param map  The map of the field. Updates to this will be reflected in
     *  the generated path.
     * @param start  The initial start point for this path planner. This can be
     *  updated later through the use of {@link #setStart(Point)}.
     * @param goal  The target location for this path planner. This cannot be
     *  changed.
     * @param radius  the obstacle growth radius to use. This is the minimum
     *  distance the path must stay away from any obstacles. This cannot be
     *  changed.
     */
    public DStarLite(ObstacleMap map, Translation2d start, Translation2d goal, double radius) {
        this.map = map;
        this.goal = new Node(goal, queue, 0, 0);
        this.start = new Node(start, queue);
        this.radius = radius;
        this.map.onAddition(this::addObstacle);
        this.map.onRemoval(this::removeObstacle);
        this.start.connect(this.goal, new LinearSegment(this.start, this.goal));
        this.goal.connect(this.start, new LinearSegment(this.goal, this.start));
        for(Obstacle obs : map.getObstacles())
            addObstacle(obs);
    }
    /**
     * Creates a DStarLite path planner.
     * 
     * <p>This constructor is equivalent to
     * {@link #DStarLite(ObstacleMap, Translation2d, Translation2d, double)},
     * but assumes the obstacle growth radius is 0.
     * 
     * @param map  The map of the field. Updates to this will be reflected in
     *  the generated path.
     * @param start  The initial start point for this path planner. This can be
     *  updated later through the use of {@link #setStart(Point)}.
     * @param goal  The target location for this path planner. This cannot be
     *  changed.
     */
    public DStarLite(ObstacleMap map, Translation2d start, Translation2d goal) {
        this(map, start, goal, 0);
    }
    /**
     * Creates a DStarLite path planner.
     * 
     * <p>This constructor is equivalent to
     * {@link #DStarLite(ObstacleMap, Translation2d, Translation2d, double)},
     * but initializes the start position to be equal to the goal position. The
     * {@link #setStart(Point)} method can be used to change the start position
     * after this object is initialized.
     * 
     * @param map  The map of the field. Updates to this will be reflected in
     *  the generated path.
     * @param goal  The target location for this path planner. This cannot be
     *  changed.
     * @param radius  the obstacle growth radius to use. This is the minimum
     *  distance the path must stay away from any obstacles. This cannot be
     *  changed.
     */
    public DStarLite(ObstacleMap map, Translation2d goal, double radius) {
        this(map, goal, goal, radius);
    }
    /**
     * Creates a DStarLite path planner.
     * 
     * <p>This constructor is equivalent to
     * {@link #DStarLite(ObstacleMap, Translation2d, Translation2d, double)},
     * but assumes the obstacle growth radius is 0 and initializes the start
     * position to be equal to the goal position. The {@link #setStart(Point)}
     * method can be used to change the start position after this object is
     * initialized.
     * 
     * @param map  The map of the field. Updates to this will be reflected in
     *  the generated path.
     * @param goal  The target location for this path planner. This cannot be
     *  changed.
     */
    public DStarLite(ObstacleMap map, Translation2d goal) {
        this(map, goal, 0);
    }

    /**
     * Sets the start position for the algorithm.
     * 
     * @param start  The new start position to use.
     */
    public void setStart(Point position) {
        start.sever();
        start = new Node(position, queue);
        Path goalEdge = new LinearSegment(start, goal);
        for(Obstacle obs : map.getObstacles()){
            for(Point endpoint : obs.getEndpoints(start, radius)){
                if(map.isClear(new LinearSegment(start, endpoint), radius, obs)){
                    Node vertex = getNode(endpoint, obs, true);
                    start.connect(vertex, new LinearSegment(position, endpoint));
                    vertex.connect(start, new LinearSegment(endpoint, position));
                }
            }
            if(goalEdge != null && !obs.isClear(goalEdge, radius))
                goalEdge = null;
        }
        if(goalEdge != null){
            start.connect(goal, goalEdge);
            goal.connect(start, goalEdge);
        }
    }

    /**
     * Finds the shortest path from the start to the goal.
     * 
     * @return  The first segment of the shortest path from the start position
     *  to the goal.
     * @see #getPath()
     */
    public Path getSegment() {
        while(queue.size() > 0 && (queue.peek().key() < start.key() || !start.isConsistent()))
            queue.poll().rectify();
        return start.getPath();
    }

    /**
     * Finds the shortest path from the start to the goal.
     * 
     * @return  The complete shortest path from the start position to the goal.
     * @see #getSegment()
     */
    public CompoundPath getPath() {
        List<Path> parts = new LinkedList<>();
        Node node = start;
        while(node != goal){
            parts.add(node.getEdge(node.getNext()));
            node = node.getNext();
        }
        return new CompoundPath(parts.toArray(new Path[0]));
    }

    /**
     * Returns an iterable over the nodes in the graph of this path planner.
     * 
     * @return  An iterable over the nodes contianed in the graph used by
     *  this D* Lite algorithm instance.
     */
    public Iterable<Node> getNodes() {
        Set<Node> res = new LinkedHashSet<>();
        res.add(goal);
        res.add(start);
        for(Map<Point, Node> map : obstacleSets.values())
            res.addAll(map.values());
        return res;
    }

    /**
     * Returns an iterable over the edges in the graph of this path planner.
     * 
     * @return  An iterable over the edges contianed in the graph used by
     *  this D* Lite algorithm instance. Two directed paths are returned for
     *  every undirected edge in the graph.
     */
    public Iterable<Path> getEdges() {
        return new Iterable<Path>() {
            @Override
            public Iterator<Path> iterator() {
                return new Iterator<Path>() {
                    Iterator<Node> nodeIter = getNodes().iterator();
                    Iterator<Path> edgeIter = null;
                    @Override
                    public boolean hasNext() {
                        return nodeIter.hasNext() || (edgeIter != null && edgeIter.hasNext());
                    }
                    @Override
                    public Path next() {
                        while(edgeIter == null || !edgeIter.hasNext())
                            edgeIter = nodeIter.next().getEdges().iterator();
                        return edgeIter.next();
                    }
                };
            }
        };
    }

    /**
     * Adds a node to the graph used by this path planner.
     * 
     * @param position  The position of the new node.
     * @param obstacle  The obstacle to associate the node with.
     * @return  True, if a new node was added, or false if the node already
     *  existed in the graph.
     */
    public boolean addNode(Point position, Obstacle obstacle) {
        NavigableMap<Point, Node> obsSet = obstacleSets.get(obstacle);
        if(position.equals(obsSet.floorKey(position)) || position.equals(obsSet.ceilingKey(position)))
            return false;
        if(obsSet.putIfAbsent(position, new Node(position, queue)) != null)
            return false;
        if(obsSet.size() > 1){
            Node node = obsSet.get(position);
            Map.Entry<Point, Node> prevEntry = obsSet.lowerEntry(position);
            Node prev = (prevEntry == null ? obsSet.lastEntry() : prevEntry).getValue();
            Map.Entry<Point, Node> nextEntry = obsSet.higherEntry(position);
            Node next = (nextEntry == null ? obsSet.firstEntry() : nextEntry).getValue();
            prev.sever(next);
            next.sever(prev);
            Path prevEdge = obstacle.edgePath(prev, node, radius);
            prev.connect(node, prevEdge);
            node.connect(prev, prevEdge.reverse());
            Path nextEdge = obstacle.edgePath(node, next, radius);
            node.connect(next, nextEdge);
            next.connect(node, nextEdge.reverse());
        }
        return true;
    }

    /**
     * Removes a node from the graph used by this path planner.
     * 
     * <p>This method will fail if the position and obstacle do not refer to
     * a valid node in the graph.
     * 
     * @param position  The position of the node to remove.
     * @param obstacle  The obstacle associated with the node.
     */
    public void dropNode(Point position, Obstacle obstacle) {
        NavigableMap<Point, Node> obsSet = obstacleSets.get(obstacle);
        Node node = getNode(position, obstacle, false);  // TODO: add check for null value
        node.sever();
        obsSet.remove(position);
        Node prev, next;
        if(obsSet.size() > 0){
            Map.Entry<Point, Node> prevEntry = obsSet.lowerEntry(position);
            prev = (prevEntry == null ? obsSet.lastEntry() : prevEntry).getValue();
            Map.Entry<Point, Node> nextEntry = obsSet.higherEntry(position);
            next = (nextEntry == null ? obsSet.firstEntry() : nextEntry).getValue();
            node.sever(next);
            node.sever(prev);
            if(obsSet.size() > 1){
                Path edge = obstacle.edgePath(prev, next, radius);
                prev.connect(next, edge);
                next.connect(prev, edge.reverse());
            }
        }
    }
    /**
     * Removes a node from the graph used by this path planner.
     * 
     * <p>This is a convience wrapper for {@link #dropNode(Point, Obstacle)}
     * when the position of the node to be removed uniquely identifies the node
     * in this graph.
     * 
     * @param position  The position of the node to remove.
     */
    public void dropNode(Point position) {
        dropNode(position, findNode(position));
    }

    /**
     * Returns the start node of this path planner.
     */
    public Node getStart() {
        return start;
    }

    /**
     * Returns the goal node of this path planner.
     */
    public Node getGoal() {
        return goal;
    }

    /**
     * Finds the obstacle associated with a node in this graph.
     * 
     * @param position  The position of the node.
     * @return  The obstacle associated with the specified node in this graph.
     */
    Obstacle findNode(Point position) {
        for(Map.Entry<Obstacle, NavigableMap<Point, Node>> entry : obstacleSets.entrySet())
            if(position.equals(entry.getValue().floorKey(position)) || position.equals(entry.getValue().ceilingKey(position)))
                return entry.getKey();
        return null;
    }

    /**
     * Returns the node associated with a specific location and obstacle.
     * 
     * @param position  The position of the node to return.
     * @param obstacle  The obstacle associated with the node.
     * @param addIfMissing  Whether to add a node with the specified position
     *  and associated obstacle if one does not already exist in this graph.
     *  If true, the added node is returned. If false, null is returned.
     * @return  Either the specified node or null, if no such node exists.
     */
    Node getNode(Point position, Obstacle obstacle, boolean addIfMissing) {
        if(addIfMissing)
            addNode(position, obstacle);
        NavigableMap<Point, Node> obsSet = obstacleSets.get(obstacle);
        if(position.equals(obsSet.floorKey(position)))
            return obsSet.floorEntry(position).getValue();
        else if(position.equals(obsSet.ceilingKey(position)))
            return obsSet.ceilingEntry(position).getValue();
        else
            return null;
    }
    /**
     * Returns the node associated with a specific location and obstacle.
     * 
     * <p>This method is identical to {@link #getNode(Point, Obstacle, boolean)}
     * with addIfMissing assumed to be true. This method will not return null.
     * 
     * @param position  The position of the node to return.
     * @param obstacle  The obstacle associated with the node.
     * @return  The specified node.
     */
    Node getNode(Point position, Obstacle obstacle) {
        return getNode(position, obstacle, true);
    }
    /**
     * Returns the node associated with a specific location and obstacle.
     * 
     * <p>This method will fail if the specified position is not associated
     * with a node in this graph. See {@link #getNode(Point, Obstacle, boolean)}
     * or {@link #getNode(Point, Obstacle)} if this is not the desired behavior.
     * 
     * @param position  The position of the node to return.
     * @return  The specified node.
     */
    Node getNode(Point position) {
        return getNode(position, findNode(position), false);
    }

    /**
     * Adds an edge to the graph used by this path planner.
     * 
     * @param a  The obstacle associated with the first end of the edge.
     * @param b  The obstacle associated with the second end of the edge.
     * @param edge  The path from the first end to the second end of the edge.
     *  Both this and the reversal of this edge will be added to the graph.
     */
    void addEdge(Obstacle a, Obstacle b, Path edge) {
        Node beg = getNode(edge.getStart(), a, true);
        Node end = getNode(edge.getEnd(), b, true);
        beg.connect(end, edge);
        end.connect(beg, edge.reverse());
    }

    /**
     * Adds an obstacle to the graph used by this path planner.
     * 
     * <p>Normally, this method will be called implicitly when an obstacle
     * is added to the map associated with this path planner.
     * 
     * @param obstacle  The obstacle to add.
     */
    public void addObstacle(Obstacle obstacle) {
        if(obstacleSets.containsKey(obstacle)) return;
        // Remove blocked connections
		for(var entry : obstacleSets.entrySet()){
            for(Node node : new ArrayList<Node>(entry.getValue().values())){
                for(Map.Entry<Node, Path> conn : new ArrayList<>(node.getConnections()))
                    if(!obstacle.isClear(conn.getValue(), radius))
                        node.sever(conn.getKey());
                if(node.getDegree() == 0)
                    dropNode(node, entry.getKey());
            }
        }
        if(start.getEdge(goal) != null && !obstacle.isClear(start.getEdge(goal), radius))
            start.sever(goal);
        obstacleSets.put(obstacle, new TreeMap<>(obstacle));
        // Connect to other obstacles
        for(Obstacle b : obstacleSets.keySet())
            for(LinearSegment edge : obstacle.getTangents(b, radius))
                if(map.isClear(edge, radius, obstacle, b))
                    addEdge(obstacle, b, edge);
        // Connect to start and goal
        for(Point endpoint : obstacle.getEndpoints(start, radius)){
            if(map.isClear(new LinearSegment(start, endpoint), radius, obstacle)){
                start.connect(getNode(endpoint, obstacle, true), new LinearSegment(start, endpoint));
                getNode(endpoint, obstacle).connect(start, new LinearSegment(endpoint, start));
            }
        }
        for(Point endpoint : obstacle.getEndpoints(goal, radius)){
            if(map.isClear(new LinearSegment(goal, endpoint), radius, obstacle)){
                goal.connect(getNode(endpoint, obstacle, true), new LinearSegment(goal, endpoint));
                getNode(endpoint, obstacle).connect(goal, new LinearSegment(endpoint, goal));
            }
        }
    }

    /**
     * Removes an obstacle from the graph used by this path planner.
     * 
     * <p>Normally, this method will be called implicitly when an obstacle
     * is fromed from the map associated with this path planner.
     * 
     * @param obstacle  The obstacle to remove.
     */
    public void removeObstacle(Obstacle obstacle) {
        for(Node node : new ArrayList<>(obstacleSets.get(obstacle).values()))
            dropNode(node, obstacle);
        obstacleSets.remove(obstacle);
        for(Obstacle a : obstacleSets.keySet()){
            for(Obstacle b : obstacleSets.keySet()){
                for(LinearSegment edge : a.getTangents(b, radius)){
                    if(map.isClear(edge, radius, a, b))
                        addEdge(a, b, edge);
                }
            }
            for(Point endpoint : a.getEndpoints(start, radius)){
                if(map.isClear(new LinearSegment(start, endpoint), radius, a)){
                    start.connect(getNode(endpoint, a, true), new LinearSegment(start, endpoint));
                    getNode(endpoint, a).connect(start, new LinearSegment(endpoint, start));
                }
            }
            for(Point endpoint : a.getEndpoints(goal, radius)){
                if(map.isClear(new LinearSegment(goal, endpoint), radius, a)){
                    goal.connect(getNode(endpoint, a, true), new LinearSegment(goal, endpoint));
                    getNode(endpoint, a).connect(goal, new LinearSegment(endpoint, goal));
                }
            }
        }
        if(start.getNext() != goal && map.isClear(new LinearSegment(start, goal), radius)){
            start.connect(goal, new LinearSegment(start, goal));
            goal.connect(start, new LinearSegment(goal, start));
        }
    }
}
