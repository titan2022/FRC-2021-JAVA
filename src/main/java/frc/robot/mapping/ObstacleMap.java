package frc.robot.mapping;

import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

/**
 * A collection of obstacles representing a map of the field.
 */
public class ObstacleMap {
    private final Set<Obstacle> obstacles = new LinkedHashSet<>();
    private final List<Consumer<Obstacle>> onAdd = new LinkedList<>();
    private final List<Consumer<Obstacle>> onDrop = new LinkedList<>();

    /**
     * Adds an obstacle to this map.
     * 
     * @param obstacle  The obstacle to add.
     */
    public void addObstacle(Obstacle obstacle) {
        obstacles.add(obstacle);
        for(Consumer<Obstacle> callback : onAdd)
            callback.accept(obstacle);
    }

    /**
     * Removes an obstacle from this map.
     * 
     * @param obstacle  The obstacle to remove.
     */
    public void removeObstacle(Obstacle obstacle) {
        if(obstacles.remove(obstacle))
            for(Consumer<Obstacle> callback : onDrop)
                callback.accept(obstacle);
    }

    /** Returns an iterable over the obstacles in this map. */
    public Iterable<Obstacle> getObstacles() {
        return obstacles;
    }

    /**
     * Determines whether a path is clear of all obstacles in this map.
     * 
     * @param path  The path to test.
     * @param radius  The obstacle growth radius to use. That is, the minimum
     *  distance the path must stay away from this obstacle to be considered
     *  clear of this obstacle.
     * @return  True, if the path is clear of all obstacles, or false,
     *  otherwise.
     * @see Obstacle#isClear(Path, double)
     */
    public boolean isClear(Path path, double radius) {
        for(Obstacle obstacle : obstacles)
            if(!obstacle.isClear(path, radius)) return false;
        return true;
    }
    /**
     * Determines whether a path is clear of all obstacles in this map.
     * 
     * <p>This method is equivalent to {@link #isClear(Path, double)} with a
     * radius of 0.
     * 
     * @param path  The path to test.
     * @return True if the path is clear of this obstacle, or false, otherwise.
     * @see Obstacle#isClear(Path)
     */
    public boolean isClear(Path path) {
        for(Obstacle obstacle : obstacles)
            if(!obstacle.isClear(path)) return false;
        return true;
    }

    /**
     * Registers a callback to be run when an obstacle is added to this map.
     * 
     * @param callback  The callback to register. When a new obstacle is added
     *  to this map, the callback will be called with the new obstacle as the
     *  sole argument.
     * @return  True, if the callback was successfully added, or false, if the
     *  callback was alreay registered.
     */
    public boolean onAddition(Consumer<Obstacle> callback) {
        return onAdd.add(callback);
    }

    /**
     * Registers a callback to be run when an obstacle is removed from this map.
     * 
     * @param callback  The callback to register. When an obstacle is removed
     *  from this map, the callback will be called with the removed obstacle as
     *  the sole argument.
     * @return  True, if the callback was successfully added, or false, if the
     *  callback was alreay registered.
     */
    public boolean onRemoval(Consumer<Obstacle> callback) {
        return onDrop.add(callback);
    }
}