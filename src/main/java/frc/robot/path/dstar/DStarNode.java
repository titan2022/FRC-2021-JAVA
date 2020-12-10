package frc.robot.path.dstar;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

import edu.wpi.first.wpilibj.geometry.Translation2d;

import frc.robot.mapping.Point;
import frc.robot.mapping.Path;

class DStarNode extends Point {
    private final Queue<DStarNode> queue;
    private double g = Double.POSITIVE_INFINITY;
    private double rhs = Double.POSITIVE_INFINITY;
    private final Map<DStarNode, Path> edges = new HashMap<>();
    private DStarNode next = null;

    DStarNode(Translation2d position, Queue<DStarNode> queue) {
        super(position);
        this.queue = queue;
    }

    Path getPath() {
        return edges.get(next);
    }

    DStarNode getNext() {
        return next;
    }

    void connect(DStarNode neighbor, Path edge) {
        if(edges.putIfAbsent(neighbor, edge) != null && edge.getLength() < edges.get(neighbor).getLength()){
            edges.put(neighbor, edge);
            if(neighbor.g + edge.getLength() < rhs){
                next = neighbor;
                rhs = neighbor.g + edge.getLength();
                update();
            }
        }
    }

    void sever(DStarNode neighbor) {
        edges.remove(neighbor);
        alert(neighbor);
        neighbor.edges.remove(this);
        neighbor.alert(this);
    }

    void sever() {
        for(DStarNode neighbor : edges.keySet()){
            neighbor.edges.remove(this);
            neighbor.alert(this);
        }
    }

    void update() {
        queue.remove(this);
        if(!isConsistent())
            queue.add(this);
    }

    void rectify() {
        if(g > rhs)
            g = rhs;
        else if(g < rhs)
            g = Double.POSITIVE_INFINITY;
        update();
        for(DStarNode neighbor : edges.keySet())
            neighbor.alert(this);
    }

    private void alert(DStarNode neighbor) {
        if(neighbor == next){
            if(!edges.containsKey(neighbor) || neighbor.g + edges.get(neighbor).getLength() > rhs){
                rhs = Double.POSITIVE_INFINITY;
                for(Map.Entry<DStarNode, Path> entry : edges.entrySet()){
                    if(entry.getKey().g + entry.getValue().getLength() < rhs){
                        next = entry.getKey();
                        rhs = entry.getKey().g + entry.getValue().getLength();
                    }
                }
                update();
            }
        }
        else if(edges.containsKey(neighbor) && neighbor.g + edges.get(neighbor).getLength() < rhs){
            next = neighbor;
            rhs = next.g + edges.get(neighbor).getLength();
            update();
        }
    }

    double key() {
        return Math.min(g, rhs);
    }

    boolean isConsistent() {
        return g == rhs;
    }
}
