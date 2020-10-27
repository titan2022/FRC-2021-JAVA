package frc.robot.path.dstar;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * A point on a 2D cartesian plane.
 */
public class Point {
  public final double x;
  public final double y;
  
  /**
   * Creates a point with specified cartesian coordinates.
   * 
   * @param x  The x coordinate of this point.
   * @param y  The y coordinate of this point.
   */
  public Point(double x, double y){
    this.x = x;
    this.y = y;
  }
  
  /**
   * Computes the angle between three points.
   * 
   * @param base  The point defining the ray the angle is measured from.
   * @param vertex  The point defining the common endpoint of the rays
   *  forming the angle to measure.
   * @param leg  The point defining the ray the angle is measured to.
   * @return  The signed angle from the ray from {@code vertex} to {@code base}
   *  to the ray from {@code vertex} to {@code leg}.
   */
  public static Rotation2d getAngle(Point base, Point vertex, Point leg) {
    return new Rotation2d(Math.atan2(leg.y - vertex.y, leg.x - vertex.x) -
      Math.atan2(base.y - vertex.y, base.x - vertex.x));
  }

  /** Returns the polar angle of this Point. */
  public Rotation2d angle() {
    return new Rotation2d(x, y);
  }
  
  /**
   * Finds the Euclidean distance from this point to another point.
   * 
   * @param dest  The point to compute the distance to.
   * @return  The Euclidean (L2) distance from this point to the point
   *  specified as the {@code dest} parameter.
   */
  public double distance(Point dest) {
    return Math.sqrt(Math.pow(x - dest.x, 2) + Math.pow(y - dest.y, 2));
  }
}