package frc.robot.path.dstar;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.PriorityQueue;
import java.util.Queue;

import org.junit.jupiter.api.Test;

import frc.robot.mapping.Point;

public class NodeTest {
    @Test
    void getDegreeTest()
    {
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(0, 0), queue);
        int degree = a.getDegree();
        int trueDegree = 0;
        assertEquals(degree, trueDegree);
    }
}
