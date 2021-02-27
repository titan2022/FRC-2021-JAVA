package frc.robot.path.dstar;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.PriorityQueue;
import java.util.Queue;

import frc.robot.mapping.Obstacle;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Point;
import frc.robot.mapping.Polygon;

/**
 * @authors Cole Plepel, Gnandeep Chintala
 */
public class DStarLiteTest {
    @Test
    void sizeTest()
    {
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(1, 1), queue);
        a.update();
        int size = queue.size();
        int trueSize = 0;
        assertEquals(trueSize, size);
        a.rectify();
        size = queue.size();
        assertEquals(trueSize, size);   
        queue.poll();
        Node b = new Node(new Point(0, 1), queue, 0, 1);
        b.update();
        size = queue.size();
        trueSize = 1;
        assertEquals(trueSize, size);
        b.rectify();
        size = queue.size();
        trueSize = 1;
        assertEquals(trueSize, size);
        queue.poll();
        Node c = new Node(new Point(1, 0), queue, 1, 0);
        c.update();
        size = queue.size();
        trueSize = 1;
        assertEquals(trueSize, size);
        c.rectify();
        size = queue.size();
        trueSize = 0;
        assertEquals(trueSize, size);
        queue.poll();
        Node d = new Node(new Point(0, 0), queue, 0, 0);
        d.update();
        size = queue.size();
        trueSize = 0;
        assertEquals(trueSize, size);
        d.rectify();
        size = queue.size();
        trueSize = 0;
        assertEquals(trueSize, size);
    }
    @Test
    void getGTest()
    {
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(1, 1), queue);
        a.update();
        a.rectify();
        double aG = a.getG();
        double aTrueG = Double.POSITIVE_INFINITY;
        assertEquals(aTrueG, aG);
        queue.poll();
        Node b = new Node(new Point(0, 1), queue, 0, 1);
        b.update();
        b.rectify();
        double bG = b.getG();
        double bTrueG = Double.POSITIVE_INFINITY;
        assertEquals(bTrueG, bG);
        queue.poll();
        Node c = new Node(new Point(1, 0), queue, 1, 0);
        c.update();
        c.rectify();
        double cG = c.getG();
        double cTrueG = 0;
        assertEquals(cTrueG, cG);
        queue.poll();
        Node d = new Node(new Point(0, 0), queue, 0, 0);
        d.update();
        d.rectify();
        double dG = d.getG();
        double dTrueG = 0;
        assertEquals(dTrueG, dG);
    }
    @Test
    void getRHSTest()
    {
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(1, 1), queue);
        a.update();
        a.rectify();
        double aRHS = a.getRhs();
        double aTrueRHS = Double.POSITIVE_INFINITY;
        assertEquals(aTrueRHS, aRHS);
        queue.poll();
        Node b = new Node(new Point(0, 1), queue, 0, 1);
        b.update();
        b.rectify();
        double bRHS = b.getRhs();
        double bTrueRHS = 1;
        assertEquals(bTrueRHS, bRHS);
        queue.poll();
        Node c = new Node(new Point(1, 0), queue, 1, 0);
        c.update();
        c.rectify();
        double cRHS = c.getRhs();
        double cTrueRHS = 0;
        assertEquals(cTrueRHS, cRHS);
        queue.poll();
        Node d = new Node(new Point(0, 0), queue, 0, 0);
        d.update();
        d.rectify();
        double dRHS = d.getRhs();
        double dTrueRHS = 0;
        assertEquals(dTrueRHS, dRHS);
    }
    @Test
    void getStartTest()
    {
        ObstacleMap map = new ObstacleMap();
        DStarLite graph = new DStarLite(map, new Point(0, 0), new Point(10, 10), 1);
        Point start = graph.getStart();
        Point trueStart = new Point(0,0);
        assertEquals(trueStart, start);
    }
    @Test 
    void getGoalTest()
    {
        ObstacleMap map = new ObstacleMap();
        DStarLite graph = new DStarLite(map, new Point(0, 0), new Point(10, 10), 1);
        Point goal = graph.getGoal();
        Point trueGoal = new Point(10,10);
        assertEquals(trueGoal, goal);   
    }
    @Test
    @SuppressWarnings("unused")
    void getNodesTest()
    {
        ObstacleMap map = new ObstacleMap();
        DStarLite graph = new DStarLite(map, new Point(0, 0), new Point(10, 10), 1);
        int nodeCount = 0;
        int trueNodeCount = 2;
        for(Node node: graph.getNodes())
        {
            nodeCount += 1;
        }
        assertEquals(trueNodeCount, nodeCount);
        graph.setStart(new Point(1,1));
        nodeCount = 0;
        trueNodeCount = 2;
        for(Node node: graph.getNodes())
        {
            nodeCount += 1;
        }
        assertEquals(trueNodeCount, nodeCount);
    }
    @Test
    void setStartTest()
    {
        ObstacleMap map = new ObstacleMap();
        DStarLite graph = new DStarLite(map, new Point(0, 0), new Point(10, 10), 1);
        graph.setStart(new Point(1,1));
        Point start = graph.getStart();
        Point trueStart = new Point(1,1);
        assertEquals(trueStart, start);
    }
    @Test 
    void getDegreeTest()
    {
        ObstacleMap map = new ObstacleMap();
        DStarLite graph = new DStarLite(map, new Point(0, 0), new Point(10, 10), 1);
        graph.setStart(new Point(1, 1));
        for(Node node : graph.getNodes()){
          int degree = node.getDegree();
          int trueDegree = 1;
          assertEquals(trueDegree, degree);
        }
    }
    @Test 
    void obstacleDegreeTest()
    {
        ObstacleMap map = new ObstacleMap();
        Obstacle first = new Polygon(new Point(1, 4), new Point(4, 1), new Point(2, 2));
        map.addObstacle(first);
        DStarLite graph = new DStarLite(map, new Point(0, 0), new Point(10, 10), 0.1);
        int startDegree = graph.getStart().getDegree();
        int trueStartDegree = 2;
        assertEquals(trueStartDegree, startDegree);
        int goalDegree = graph.getGoal().getDegree();
        int trueGoalDegree = 2;
        assertEquals(trueGoalDegree, goalDegree);
        Obstacle second = new Polygon(new Point(3, 3), new Point(6, 0), new Point(4, 4));
        map.addObstacle(second);
        startDegree = graph.getStart().getDegree();
        trueStartDegree = 3;
        assertEquals(trueStartDegree, startDegree);
        goalDegree = graph.getGoal().getDegree();
        trueGoalDegree = 3;
        assertEquals(trueGoalDegree, goalDegree);
        map.removeObstacle(first);
        startDegree = graph.getStart().getDegree();
        trueStartDegree = 2;
        assertEquals(trueStartDegree, startDegree);
        goalDegree = graph.getGoal().getDegree();
        trueGoalDegree = 2;
        assertEquals(trueGoalDegree, goalDegree);
        map.removeObstacle(second);
        startDegree = graph.getStart().getDegree();
        trueStartDegree = 1;
        assertEquals(trueStartDegree, startDegree);
        goalDegree = graph.getGoal().getDegree();
        trueGoalDegree = 1;
        assertEquals(trueGoalDegree, goalDegree);
    }
    @Test
    void nextNodeTest()
    {
        ObstacleMap map = new ObstacleMap();
        Obstacle first = new Polygon(new Point(1, 4), new Point(4, 1), new Point(2, 2));
        map.addObstacle(first);
        DStarLite graph = new DStarLite(map, new Point(0, 0), new Point(10, 10), 0.1);
        Node nextNode = graph.getGoal().getNext();
        Node trueNode = null;
        assertEquals(trueNode, nextNode);
        Obstacle second = new Polygon(new Point(3, 3), new Point(6, 0), new Point(4, 4));
        map.addObstacle(second);
        map.removeObstacle(first);
        map.removeObstacle(second);
        nextNode = graph.getStart().getNext();
        trueNode = graph.getGoal();
        assertEquals(trueNode, nextNode);
    }
}
