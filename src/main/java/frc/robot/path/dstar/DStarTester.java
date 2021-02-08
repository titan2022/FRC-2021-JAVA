package frc.robot.path.dstar;

import java.util.PriorityQueue;
import java.util.Queue;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.LinearSegment;
import frc.robot.mapping.Obstacle;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Point;
import frc.robot.mapping.Polygon;

public class DStarTester extends CommandBase {
    @Override
    public void initialize() {
        System.out.print("Node methods: ");
        System.out.println(testNode());
        System.out.print("D* Lite primitives: ");
        System.out.println(testPrimitives());
        System.out.print("D* Lite graph getters and setters: ");
        System.out.println(testGraphGetters());
        System.out.print("Obstacle addition/removal: ");
        System.out.println(testObstacles());
    }

    @Override
    public void execute() {
        return;
    }

    public boolean testNode() {
        boolean truth = true;
        boolean allPass = true;
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(0, 0), queue);
        truth = a.getDegree() == 0;
        allPass &= truth;
        truth = a.key() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        truth = a.isConsistent();
        allPass &= truth;
        Node b = new Node(new Point(1, 0), queue, 0, 0);
        truth = b.key() == 0;
        allPass &= truth;
        truth = b.isConsistent();
        allPass &= truth;
        a.connect(b, new LinearSegment(a, b));
        b.connect(a, new LinearSegment(b, a));
        truth = a.getDegree() == 1;
        allPass &= truth;
        truth = b.getDegree() == 1;
        allPass &= truth;
        truth = a.getG() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        truth = b.getG() == 0;
        allPass &= truth;
        truth = a.getRhs() == 1;
        allPass &= truth;
        truth = b.getRhs() == 0;
        allPass &= truth;
        truth = a.getNext() == b;
        allPass &= truth;
        truth = b.getNext() == null;
        allPass &= truth;
        truth = queue.contains(a);
        allPass &= truth;
        truth = !queue.contains(b);
        allPass &= truth;

        Node c = new Node(new Point(0, 2), queue, 5, Double.POSITIVE_INFINITY);
        truth = !c.isConsistent();
        allPass &= truth;
        a.connect(c, new LinearSegment(a, c));
        c.connect(a, new LinearSegment(c, a));
        truth = a.getDegree() == 2;
        allPass &= truth;
        truth = c.getDegree() == 1;
        allPass &= truth;
        truth = c.getRhs() == a.getG() + 2;
        allPass &= truth;
        truth = c.getRhs() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        a.sever(b);
        truth = a.getDegree() == 1;
        allPass &= truth;
        truth = b.getDegree() == 0;
        allPass &= truth;
        truth = a.getRhs() == c.getG() + 2;
        allPass &= truth;
        return allPass;
    }

    public boolean testPrimitives() {
        boolean truth = true;
        boolean allPass = true;
        Queue<Node> queue = new PriorityQueue<>();
        Node a = new Node(new Point(1, 1), queue);
        a.update();
        truth = queue.size() == 0;
        allPass &= truth;
        a.rectify();
        truth = queue.size() == 0;
        allPass &= truth;
        truth = a.getG() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        truth = a.getRhs() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        queue.poll();
        Node b = new Node(new Point(0, 1), queue, 0, 1);
        b.update();
        truth = queue.size() == 1;
        allPass &= truth;
        b.rectify();
        truth = queue.size() == 1;
        allPass &= truth;
        truth = b.getG() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        truth = b.getRhs() == 1;
        allPass &= truth;
        queue.poll();
        Node c = new Node(new Point(1, 0), queue, 1, 0);
        c.update();
        truth = queue.size() == 1;
        allPass &= truth;
        c.rectify();
        truth = queue.size() == 0;
        allPass &= truth;
        truth = c.getG() == 0;
        allPass &= truth;
        truth = c.getRhs() == 0;
        allPass &= truth;
        queue.poll();
        Node d = new Node(new Point(0, 0), queue, 0, 0);
        d.update();
        truth = queue.size() == 0;
        allPass &= truth;
        d.rectify();
        truth = queue.size() == 0;
        allPass &= truth;
        truth = d.getG() == 0;
        allPass &= truth;
        truth = d.getRhs() == 0;
        allPass &= truth;
        return allPass;
    }

    public boolean testGraphGetters() {
        boolean truth = true;
        boolean allPass = true;
        ObstacleMap map = new ObstacleMap();
        DStarLite graph = new DStarLite(map, new Point(0, 0), new Point(10, 10), 1);
        truth = graph.getStart().equals(new Point(0, 0));
        allPass &= truth;
        truth = graph.getGoal().equals(new Point(10, 10));
        allPass &= truth;
        int n = 0;
        for(Node node : graph.getNodes())
            n += 1;
        truth = n == 2;
        allPass &= truth;
        graph.setStart(new Point(1, 1));
        truth = graph.getStart().equals(new Point(1, 1));
        allPass &= truth;
        n = 0;
        for(Node node : graph.getNodes()){
            n += 1;
            truth = node.getDegree() == 1;
            allPass &= truth;
        }
        truth = n == 2;
        allPass &= truth;
        return allPass;
    }

    public boolean testObstacles() {
        boolean truth = true;
        boolean allPass = true;
        ObstacleMap map = new ObstacleMap();
        Obstacle first = new Polygon(new Point(1, 4), new Point(4, 1), new Point(2, 2));
        map.addObstacle(first);
        DStarLite graph = new DStarLite(map, new Point(0, 0), new Point(10, 10), 0.1);
        truth = graph.getStart().getDegree() == 2;
        allPass &= truth;
        truth = graph.getGoal().getDegree() == 2;
        allPass &= truth;
        truth = graph.getGoal().getNext() == null;
        allPass &= truth;
        Obstacle second = new Polygon(new Point(3, 3), new Point(6, 0), new Point(4, 4));
        map.addObstacle(second);
        truth = graph.getGoal().getDegree() == 3;
        allPass &= truth;
        truth = graph.getStart().getDegree() == 3;
        allPass &= truth;
        map.removeObstacle(first);
        truth = graph.getStart().getDegree() == 2;
        allPass &= truth;
        truth = graph.getGoal().getDegree() == 2;
        allPass &= truth;
        map.removeObstacle(second);
        truth = graph.getStart().getNext() == graph.getGoal();
        allPass &= truth;
        truth = graph.getStart().getDegree() == 1;
        allPass &= truth;
        truth = graph.getGoal().getDegree() == 1;
        allPass &= truth;
        return allPass;
    }
}
