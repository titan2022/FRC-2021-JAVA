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

public class DStarGraph {
    private final Queue<DStarNode> queue = new PriorityQueue<DStarNode>();
    private final DStarNode goal;
    private DStarNode start;
    public final ObstacleMap map;
    public final double radius;
    private Map<Obstacle, NavigableMap<Point, DStarNode>> obstacleSets = new HashMap<>();

    public DStarGraph(ObstacleMap map, Translation2d start, Translation2d goal, double radius) {
        this.map = map;
        this.goal = new DStarNode(goal, queue, 0, 0);
        this.start = new DStarNode(start, queue);
        this.radius = radius;
        this.map.onAddition(this::addObstacle);
        this.map.onRemoval(this::removeObstacle);
        this.start.connect(this.goal, new LinearSegment(this.start, this.goal));
        this.goal.connect(this.start, new LinearSegment(this.goal, this.start));
        for(Obstacle obs : map.getObstacles())
            addObstacle(obs);
    }
    public DStarGraph(ObstacleMap map, Translation2d start, Translation2d goal) {
        this(map, start, goal, 0);
    }
    public DStarGraph(ObstacleMap map, Translation2d goal, double radius) {
        this(map, goal, goal, radius);
    }
    public DStarGraph(ObstacleMap map, Translation2d goal) {
        this(map, goal, 0);
    }

    public void setStart(Point position) {
        start.sever();
        start = new DStarNode(position, queue);
        Path goalEdge = new LinearSegment(start, goal);
        for(Obstacle obs : map.getObstacles()){
            for(Point endpoint : obs.getEndpoints(start, radius)){
                if(map.isClear(new LinearSegment(start, endpoint), radius, obs)){
                    addNode(endpoint, obs);
                    DStarNode vertex = obstacleSets.get(obs).get(endpoint);  // TODO: change to getNode
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

    public Path getSegment() {
        while(queue.size() > 0 && (queue.peek().key() < start.key() || !start.isConsistent()))
            queue.poll().rectify();
        return start.getPath();
    }

    public CompoundPath getPath() {
        List<Path> parts = new LinkedList<>();
        DStarNode node = start;
        while(node != goal){
            parts.add(node.getEdge(node.getNext()));
            node = node.getNext();
        }
        return new CompoundPath(parts.toArray(new Path[0]));
    }

    public Iterable<DStarNode> getNodes() {
        Set<DStarNode> res = new LinkedHashSet<>();
        res.add(goal);
        res.add(start);
        for(Map<Point, DStarNode> map : obstacleSets.values())
            res.addAll(map.values());
        return res;
    }

    public Iterable<Path> getEdges() {
        return new Iterable<Path>() {
            @Override
            public Iterator<Path> iterator() {
                return new Iterator<Path>() {
                    Iterator<DStarNode> nodeIter = getNodes().iterator();
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

    public boolean addNode(Point position, Obstacle obstacle) {
        NavigableMap<Point, DStarNode> obsSet = obstacleSets.get(obstacle);
        if(position.equals(obsSet.floorKey(position)) || position.equals(obsSet.ceilingKey(position)))
            return false;
        if(obsSet.putIfAbsent(position, new DStarNode(position, queue)) != null)
            return false;
        if(obsSet.size() > 1){
            DStarNode node = obsSet.get(position);
            Map.Entry<Point, DStarNode> prevEntry = obsSet.lowerEntry(position);
            DStarNode prev = (prevEntry == null ? obsSet.lastEntry() : prevEntry).getValue();
            Map.Entry<Point, DStarNode> nextEntry = obsSet.higherEntry(position);
            DStarNode next = (nextEntry == null ? obsSet.firstEntry() : nextEntry).getValue();
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

    public void dropNode(Point position, Obstacle obstacle) {
        NavigableMap<Point, DStarNode> obsSet = obstacleSets.get(obstacle);
        DStarNode node = obsSet.get(position);  // TODO: change to getNode
        node.sever();
        obsSet.remove(position);
        DStarNode prev, next;
        if(obsSet.size() > 0){
            Map.Entry<Point, DStarNode> prevEntry = obsSet.lowerEntry(position);
            prev = (prevEntry == null ? obsSet.lastEntry() : prevEntry).getValue();
            Map.Entry<Point, DStarNode> nextEntry = obsSet.higherEntry(position);
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
    public void dropNode(Point position) {
        dropNode(position, findNode(position));
    }

    public DStarNode getStart() {
        return start;
    }

    public DStarNode getGoal() {
        return goal;
    }

    Obstacle findNode(Point position) {
        for(Map.Entry<Obstacle, NavigableMap<Point, DStarNode>> entry : obstacleSets.entrySet())
            if(position.equals(entry.getValue().floorKey(position)) || position.equals(entry.getValue().ceilingKey(position)))
                return entry.getKey();
        return null;
    }

    DStarNode getNode(Point position, Obstacle obstacle, boolean addIfMissing) {
        if(addIfMissing)
            addNode(position, obstacle);
        NavigableMap<Point, DStarNode> obsSet = obstacleSets.get(obstacle);
        if(position.equals(obsSet.floorKey(position)))
            return obsSet.floorEntry(position).getValue();
        else if(position.equals(obsSet.ceilingKey(position)))
            return obsSet.ceilingEntry(position).getValue();
        else
            return null;
    }
    DStarNode getNode(Point position, Obstacle obstacle) {
        return getNode(position, obstacle, true);
    }
    DStarNode getNode(Point position) {
        return getNode(position, findNode(position), false);
    }

    void addEdge(Obstacle a, Obstacle b, Path edge) {
        addNode(edge.getStart(), a);
        addNode(edge.getEnd(), b);
        DStarNode beg = obstacleSets.get(a).get(edge.getStart());  // TODO: change to getNode
        DStarNode end = obstacleSets.get(b).get(edge.getEnd());  // TODO: change to getNode
        beg.connect(end, edge);
        end.connect(beg, edge.reverse());
    }

    void addObstacle(Obstacle obstacle) {
        if(obstacleSets.containsKey(obstacle)) return;
        // Remove blocked connections
		for(var entry : obstacleSets.entrySet()){
            for(DStarNode node : new ArrayList<DStarNode>(entry.getValue().values())){
                for(Map.Entry<DStarNode, Path> conn : new ArrayList<>(node.getConnections()))
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
                addNode(endpoint, obstacle);
                start.connect(obstacleSets.get(obstacle).get(endpoint), new LinearSegment(start, endpoint));  // TODO: change to getNode
                obstacleSets.get(obstacle).get(endpoint).connect(start, new LinearSegment(endpoint, start));  // TODO: change to getNode
            }
        }
        for(Point endpoint : obstacle.getEndpoints(goal, radius)){
            if(map.isClear(new LinearSegment(goal, endpoint), radius, obstacle)){
                addNode(endpoint, obstacle);
                goal.connect(obstacleSets.get(obstacle).get(endpoint), new LinearSegment(goal, endpoint));  // TODO: change to getNode
                obstacleSets.get(obstacle).get(endpoint).connect(goal, new LinearSegment(endpoint, goal));  // TODO: change to getNode
            }
        }
    }

    public void removeObstacle(Obstacle obstacle) {
        for(DStarNode node : new ArrayList<>(obstacleSets.get(obstacle).values()))
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
                    addNode(endpoint, a);
                    start.connect(obstacleSets.get(a).get(endpoint), new LinearSegment(start, endpoint));  // TODO: change to getNode
                    obstacleSets.get(a).get(endpoint).connect(start, new LinearSegment(endpoint, start));  // TODO: change to getNode
                }
            }
            for(Point endpoint : a.getEndpoints(goal, radius)){
                if(map.isClear(new LinearSegment(goal, endpoint), radius, a)){
                    addNode(endpoint, a);
                    goal.connect(obstacleSets.get(a).get(endpoint), new LinearSegment(goal, endpoint));  // TODO: change to getNode
                    obstacleSets.get(a).get(endpoint).connect(goal, new LinearSegment(endpoint, goal));  // TODO: change to getNode
                }
            }
        }
        if(start.getNext() != goal && map.isClear(new LinearSegment(start, goal), radius)){
            start.connect(goal, new LinearSegment(start, goal));
            goal.connect(start, new LinearSegment(goal, start));
        }
    }
}
