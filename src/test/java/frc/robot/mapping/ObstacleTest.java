package frc.robot.mapping;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;


public class ObstacleTest {
    @Test
    public void isClearTest()
    {
        Obstacle obs = new Polygon(new Point(0,0), new Point(1,0), new Point(0,1), new Point(1,1));
        LinearSegment line = new LinearSegment(new Point(-4,0), new Point(-3,0));
        boolean clear = obs.isClear(line, 1.0);
        boolean trueClear = true;
        assertEquals(clear, trueClear);
    }

    public void getEndpointsTest()
    {
        Obstacle obs = new Polygon(new Point(0,0), new Point(1,0), new Point(0,1), new Point(1,1));
        Iterable<Point> endpoints = obs.getEndpoints(new Point(-1,0), 1.0);
        //Iterable<Point> trueEndpoints = new Iterable<Point>();
        //assertEquals(endpoints, trueEndpoints);
    }

    public void getTangentsTest()
    {
        Obstacle obs = new Polygon(new Point(0,0), new Point(1,0), new Point(0,1), new Point(1,1));
        Iterable<LinearSegment> tangents = obs.getTangents(obs, 1.0);
        //Iterable<LinearSegment> trueTangents = new Iterable<LinearSegment>();
        //assertEquals(tangents, trueTangents);
    }

    public void translateByTest()
    {

    }

    public void rotateByTest()
    {

    }

    @Test
    public void isEndpointTest()
    {
        Obstacle obs = new Polygon(new Point(0,0), new Point(1,0), new Point(0,1), new Point(1,1));
        boolean endpoint = obs.isEndpoint(new Point(0,0), new Point(0,0), 1.0);
        boolean trueEndpoint = true;
        assertEquals(endpoint, trueEndpoint);
    }

    public void getPerimeterTest()
    {

    }

    public void edgePathTest()
    {

    }

    public void getBoundaryTest()
    {
        
    }
}
