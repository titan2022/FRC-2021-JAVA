package frc.robot.mapping;


import org.junit.jupiter.api.Test;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import static org.junit.jupiter.api.Assertions.assertEquals;
public class ObstacleMapTest {
    @Test
    public void isClearTest() {
    }
    @Test
    public void getObstaclesTest()
    {
        Obstacle poly = new Polygon(new Point(-2,-2), new Point (-2,2), new Point (2,2), new Point(2,-2));
        ObstacleMap map = new ObstacleMap();
        map.addObstacle(poly);
        Iterable<Obstacle> obs = map.getObstacles();
        assertEquals(obs, poly);
    }
    boolean bool = false;
    void myMethod(Obstacle o) {
        bool = true;
    }
    @Test
    public void onAdditionTest()
    {
        Obstacle poly = new Polygon(new Point(-2,-2), new Point (-2,2), new Point (2,2), new Point(2,-2));
        ObstacleMap map = new ObstacleMap();
        boolean add = map.onAddition(this::myMethod);
        map.addObstacle(poly);
        assertEquals(bool, true);
    }

    boolean bool2 = false;
    void myMethod2(Obstacle o) {
        bool2 = true;
    }
    @Test
    public void onRemovalTest()
    {
        Obstacle poly2 = new Polygon(new Point(-2,-2), new Point (-2,2), new Point (2,2), new Point(2,-2));
        ObstacleMap map2 = new ObstacleMap();
        boolean remove = map2.onRemoval(this::myMethod2);
        map2.addObstacle(poly2);
        map2.removeObstacle(poly2);
        assertEquals(bool2, true);
    }
}