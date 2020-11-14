package frc.robot.mapping;

import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

public class ObstacleMap {
    private final Set<Obstacle> obstacles = new LinkedHashSet<>();
    private final List<Consumer<Obstacle>> onAdd = new LinkedList<>();
    private final List<Consumer<Obstacle>> onDrop = new LinkedList<>();

    public void addObstacle(Obstacle obstacle) {
        obstacles.add(obstacle);
        for(Consumer<Obstacle> callback : onAdd)
            callback.accept(obstacle);
    }

    public void removeObstacle(Obstacle obstacle) {
        if(obstacles.remove(obstacle))
            for(Consumer<Obstacle> callback : onDrop)
                callback.accept(obstacle);
    }

    public Iterable<Obstacle> getObstacles() {
        return obstacles;
    }

    public boolean isClear(Path path, double radius) {
        for(Obstacle obstacle : obstacles)
            if(!obstacle.isClear(path, radius)) return false;
        return true;
    }
    public boolean isClear(Path path) {
        for(Obstacle obstacle : obstacles)
            if(!obstacle.isClear(path)) return false;
        return true;
    }

    public boolean onAddition(Consumer<Obstacle> callback) {
        return onAdd.add(callback);
    }

    public boolean onRemoval(Consumer<Obstacle> callback) {
        return onDrop.add(callback);
    }
}