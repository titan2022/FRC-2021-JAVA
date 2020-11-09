package frc.robot.path.dstar;

import java.util.List;
import java.util.LinkedList;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.wpilibjTemp.Field2d;

public class DStarDemoCommand extends CommandBase {
  DStarLite planner;
  Field2d robot;
  double radius;
  double speed;

  public DStarDemoCommand(double speed, double radius) {
    this.radius = radius;
    this.speed = speed;
  }
  public DStarDemoCommand(double speed) {
    this(speed, 1.0);
  }
  public DStarDemoCommand() {
    this(0.02);
  }

  public void initialize() {
    robot = new Field2d();
    planner = new DStarLite(new Node(0, 0), new Node(7, 7));
    List<Point> verts;
    verts = new LinkedList<Point>();
    verts.add(new Point(-10, 1));
    verts.add(new Point(2, 1));
    verts.add(new Point(1, 2));
    planner.addObstacle(new Obstacle(verts));
    verts = new LinkedList<Point>();
    verts.add(new Point(3, 5));
    verts.add(new Point(5, 7));
    verts.add(new Point(12, 4));
    verts.add(new Point(11, 2));
    verts.add(new Point(7, 3));
    planner.addObstacle(new Obstacle(verts));
  }

  public void execute() {
    planner.setStart(new Node(robot.getRobotPose().getTranslation()));
    Segment segment = planner.getSegment();
    robot.setRobotPose(
        new Pose2d(segment.getPos(speed), segment.getRotation(speed)));
  }
}