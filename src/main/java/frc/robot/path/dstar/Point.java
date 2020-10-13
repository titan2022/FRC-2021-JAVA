package frc.robot.path.dstar;

public class Point {
  public final double x;
  public final double y;
  
  public Point(double x, double y){
    this.x = x;
    this.y = y;
  }
  
  public static double getAngle(Point base, Point vertex, Point leg) {
    return (Math.atan2(leg.y - vertex.y, leg.x - vertex.x) -
      Math.atan2(base.y - vertex.y, base.x - vertex.x)) % Math.PI;
  }
  
  public double distance(Point dest) {
    return Math.sqrt(Math.pow(x - dest.x, 2) + Math.pow(y - dest.y, 2));
  }
}