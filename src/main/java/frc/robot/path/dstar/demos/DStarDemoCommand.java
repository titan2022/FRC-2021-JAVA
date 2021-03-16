package frc.robot.path.dstar.demos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.path.dstar.DStarLite;
import frc.robot.mapping.Point;
import frc.robot.mapping.Polygon;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Path;

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
    SmartDashboard.putData("Field", robot);
    ObstacleMap map = new ObstacleMap();
    planner = new DStarLite(map, new Point(0, 0), new Point(6, 8), radius);
    map.addObstacle(new Polygon(new Point(-10, 1), new Point(2, 1), new Point(1, 2)));
    map.addObstacle(new Polygon(
      new Point(3, 5),
      new Point(5, 7),
      new Point(12, 4),
      new Point(11, 2),
      new Point(7, 3))
    );
  }

  public void execute() {
    planner.setStart(new Point(robot.getRobotPose().getTranslation()));
    Path segment = planner.getPath();
    robot.setRobotPose(
        new Pose2d(segment.getPos(speed), segment.getRotation(speed)));
  }
}