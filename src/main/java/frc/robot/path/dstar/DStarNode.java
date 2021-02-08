package frc.robot.path.dstar;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

import edu.wpi.first.wpilibj.geometry.Translation2d;

import frc.robot.mapping.Point;
import frc.robot.mapping.Path;

/** A node along a possible D* Lite path */
public class DStarNode extends Point implements Comparable<DStarNode> {
    private final Queue<DStarNode> queue;
    private double g = Double.POSITIVE_INFINITY;
    private double rhs = Double.POSITIVE_INFINITY;
    private final Map<DStarNode, Path> edges = new HashMap<>();
    private DStarNode next = null;

    /**
     * Creates a node at a specific position.
     * 
     * @param position  The position of this node.
     * @param queue  The queue to associate this node with.
     */
    DStarNode(Translation2d position, Queue<DStarNode> queue) {
        super(position);
        this.queue = queue;
    }
    /**
     * Creates a node at a specific position.
     * 
     * @param position  The position of this node.
     * @param queue  The queue to associate this node with.
     * @param g  The initial g value of this node.
     * @param rhs  The initial rhs value of this node.
     * @see #getG()
     * @see #getRhs()
     */
    DStarNode(Translation2d position, Queue<DStarNode> queue, double g, double rhs) {
        this(position, queue);
        this.g = g;
        this.rhs = rhs;
    }

    /**
     * Returns the next edge on the minimum cost path through this node.
     */
    public Path getPath() {
        return edges.get(next);
    }

    /**
     * Returns the next node on the minimum cost path through this node.
     */
    public DStarNode getNext() {
        return next;
    }

    /**
     * Connects this node to another,
     * 
     * <p>This method does not automatically create the reverse connection.
     * 
     * @param neighbor  The new neighbor of this node.
     * @param edge  The edge conning this node to the new neighbor. If this
     *  node is already connected to the specified neighbor, this edge is used
     *  only is it is shorter than the existing edge.
     */
    public void connect(DStarNode neighbor, Path edge) {
        if(neighbor == this)
            return;
        if(edges.putIfAbsent(neighbor, edge) != null && edge.getLength() < edges.get(neighbor).getLength())
            edges.put(neighbor, edge);
        if(neighbor.g + edges.get(neighbor).getLength() < rhs){
            next = neighbor;
            rhs = neighbor.g + edge.getLength();
            update();
        }
    }

    /**
     * Severs the edge between this node an one of its neighbors.
     * 
     * <p>This method severs edges with the specified node in both directions.
     * 
     * @param neighbor  The neighbor to disconnect this node from.
     */
    public void sever(DStarNode neighbor) {
        edges.remove(neighbor);
        alert(neighbor);
        neighbor.edges.remove(this);
        neighbor.alert(this);
    }
    /**
     * Severs all edges to this node.
     */
    public void sever() {
        for(DStarNode neighbor : edges.keySet()){
            neighbor.edges.remove(this);
            neighbor.alert(this);
        }
    }

    /**
     * Updates the position of this node in the queue.
     * 
     * <p>Normally, this method is called during the execution of other node
     * methods.
     */
    public void update() {
        queue.remove(this);
        if(!isConsistent())
            queue.add(this);
    }

    /**
     * Updates the g estimate of the distance of this node from the goal.
     */
    public void rectify() {
        if(g > rhs)
            g = rhs;
        else if(g < rhs)
            g = Double.POSITIVE_INFINITY;
        update();
        for(DStarNode neighbor : edges.keySet())
            neighbor.alert(this);
    }

    /**
     * Updates the rhs estimate of the distance of this node from the goal.
     * 
     * @param neighbor  The neighbor of this goal to update the rhs estimate
     *  in response to.
     */
    private void alert(DStarNode neighbor) {
        if(neighbor == next){
            if(!edges.containsKey(neighbor) || neighbor.g + edges.get(neighbor).getLength() > rhs){
                rhs = Double.POSITIVE_INFINITY;
                next = null;
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

    public double key() {
        return Math.min(g, rhs);
    }

    public boolean isConsistent() {
        return g == rhs;
    }

    public double getG() {
        return g;
    }

    public double getRhs() {
        return rhs;
    }

    public int getDegree() {
        return edges.size();
    }

    /**
     * Returns a view of the neighbors of this node.
     */
    public Collection<DStarNode> getNeighbors() {
        return edges.keySet();
    }

    /**
     * Returns a view of the edges of this node.
     */
    public Collection<Path> getEdges() {
        return edges.values();
    }

    /**
     * Returns a view of the neighbors and edges of this node.
     * 
     * @return  A collection of map entries mapping the neighbors of this
     *  node to the edges associated with them.
     */
    public Collection<Map.Entry<DStarNode, Path>> getConnections() {
        return edges.entrySet();
    }

    
    /**
     * Returns the edge from this node associated with one of its neighbors.
     */
    public Path getEdge(DStarNode neighbor) {
        return edges.get(neighbor);
    }

    @Override
    public int compareTo(DStarNode other) {
        return (int) Math.signum(key() - other.key());
    }
}
