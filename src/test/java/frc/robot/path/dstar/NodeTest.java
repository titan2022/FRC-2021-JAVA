package frc.robot.path.dstar;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.PriorityQueue;
import java.util.Queue;

import frc.robot.mapping.LinearSegment;
import frc.robot.mapping.Point;

/**
 * @authors Cole Plepel, Gnandeep Chintala
 */
public class NodeTest {
    @Test
    void getDegreeTest()
    {
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(0, 0), queue);
        Node b = new Node(new Point(1, 0), queue, 0, 0);
        int aDegree = a.getDegree();
        int aTrueDegree = 0;
        assertEquals(aDegree, aTrueDegree);
        a.connect(b, new LinearSegment(a, b));
        b.connect(a, new LinearSegment(b, a));
        aDegree = a.getDegree();
        aTrueDegree = 1;
        assertEquals(aTrueDegree, aDegree);
        int bDegree = b.getDegree();
        int bTrueDegree = 1;
        assertEquals(bTrueDegree, bDegree);
        Node c = new Node(new Point(0, 2), queue, 5, Double.POSITIVE_INFINITY);
        a.connect(c, new LinearSegment(a, c));
        c.connect(a, new LinearSegment(c, a));
        aDegree = a.getDegree();
        aTrueDegree = 2;
        int cDegree = c.getDegree();
        int cTrueDegree = 1;
        assertEquals(aTrueDegree, aDegree);
        assertEquals(cTrueDegree, cDegree);
        a.sever(b);
        aDegree = a.getDegree();
        aTrueDegree = 1;
        bDegree = b.getDegree();
        bTrueDegree = 0;
        assertEquals(aTrueDegree, aDegree);
        assertEquals(bTrueDegree, bDegree);
    }
    @Test
    void keyTest()
    {
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(0, 0), queue);
        Node b = new Node(new Point(1, 0), queue, 0, 0);
        Node c = new Node(new Point(0, 1), queue, 0, 1);
        Node d = new Node(new Point(1, 1), queue, 1, 0);
        double aKey = a.key();
        double aTrueKey = Double.POSITIVE_INFINITY;
        assertEquals(aTrueKey, aKey);
        double bKey = b.key();
        double bTrueKey = 0;
        assertEquals(bTrueKey, bKey);
        double cKey = c.key();
        double cTrueKey = 0;
        assertEquals(cTrueKey, cKey);
        double dKey = d.key();
        double dTrueKey = 0;
        assertEquals(dTrueKey, dKey);
    }
    @Test
    void getGTest()
    {
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(0, 0), queue);
        Node b = new Node(new Point(1, 0), queue, 0, 0);
        double aG = a.getG();
        double bG = b.getG();
        double aTrueG = Double.POSITIVE_INFINITY;
        double bTrueG = 0;
        assertEquals(aTrueG, aG);
        assertEquals(bTrueG, bG);
        
    }
    @Test
    void getRHSTest()
    {
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(0, 0), queue);
        Node b = new Node(new Point(1, 0), queue, 0, 0);
        a.connect(b, new LinearSegment(a, b));
        b.connect(a, new LinearSegment(b, a));
        double aRHS = a.getRhs();
        double bRHS = b.getRhs();
        double aTrueRHS = 1;
        double bTrueRHS = 0;
        assertEquals(aTrueRHS, aRHS);
        assertEquals(bTrueRHS, bRHS);
        Node c = new Node(new Point(0, 2), queue, 5, Double.POSITIVE_INFINITY);
        a.connect(c, new LinearSegment(a, c));
        c.connect(a, new LinearSegment(c, a));
        double cRHS = c.getRhs();
        double cTrueRHS = a.getG()+2;
        assertEquals(cTrueRHS, cRHS);
        cTrueRHS = Double.POSITIVE_INFINITY;
        assertEquals(cTrueRHS, cRHS);
        a.sever(b);
        aRHS = a.getRhs();
        aTrueRHS = c.getG()+2;
        assertEquals(aTrueRHS, aRHS);
    }
    @Test
    void getNextTest()
    {
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(0, 0), queue);
        Node b = new Node(new Point(1, 0), queue, 0, 0);
        a.connect(b, new LinearSegment(a, b));
        b.connect(a, new LinearSegment(b, a));
        Node aNext = a.getNext();
        Node bNext = b.getNext();
        Node aTrueNext = b;
        Node bTrueNext = null;
        assertEquals(aTrueNext, aNext);
        assertEquals(bTrueNext, bNext);
    }
}
