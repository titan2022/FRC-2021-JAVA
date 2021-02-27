package frc.robot.mapping;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * @author Irene Liu
 */
public class ObstacleMapTest {
    @Test
    public void isClearTest() {
        Obstacle poly = new Polygon(new Point(-2,-2), new Point (-2,2), new Point (2,2), new Point(2,-2));
        ObstacleMap map = new ObstacleMap();
        map.addObstacle(poly);
        LinearSegment p = new LinearSegment(new Point(-3,0), new Point(0,4)); 
        boolean clear = map.isClear(p,2.0);
        assertEquals(clear, false);
    }
    @Test
    public void getObstaclesTest()
    {
        Obstacle poly = new Polygon(new Point(-2,-2), new Point (-2,2), new Point (2,2), new Point(2,-2));
        ObstacleMap map = new ObstacleMap();
        map.addObstacle(poly);
        Iterable<Obstacle> obs = map.getObstacles();
        assertEquals(obs.iterator().next(), poly);
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