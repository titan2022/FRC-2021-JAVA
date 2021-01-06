package frc.robot.mapping;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * A circular arc path.
 */
public class CircularArc implements Path {
  protected final Point center;
  protected final double theta;
  protected final double radius;
  protected final Point off;

  /**
   * Creates a new CircularArc with the specified center, start, and angle.
   * 
   * @param center  The center of the circle this arc is constructed from.
   * @param start  The start point of this path.
   * @param theta  The angle, in radiuans, subtended by this arc. Positive
   *  values refer to counterclockwise rotation, while negative values refer to
   *  clockwise rotation. Values greater in magnitude than 2pi represent arcs
   *  that loop around the circle multiple times before terminating.
   */
  public CircularArc(Point center, Point start, double theta) {
    this.center = center;
    this.theta = theta;
    radius = center.getDistance(start);
    off = start.minus(center);
  }
  /**
   * Creates a new CircularArc from start, end, and center points.
   * 
   * <p>The direction of rotation is inferred such that measure of the angle
   * subtended by this arc is never more than pi radians (90 degrees).
   * 
   * @param start  The start point of this path.
   * @param center  The center of the arc this arc is constructed from.
   * @param end  The terminating point of this path. This point and the start
   *  point must be equidistant from the center point, or an
   *  IllegalArgumentException is thrown.
   */
  public CircularArc(Point start, Point center, Point end) {
    if(center.getDistance(start) != center.getDistance(end))
      throw new IllegalArgumentException("start and end must be equidistant from center.");
    this.center = center;
    theta = Point.getAngle(start, center, end).getRadians();
    radius = center.getDistance(start);
    off = start.minus(start);
  }
  /**
   * Constructs a new CircularArc from its center, angle, initial offset.
   * 
   * @param center  The center of the circle this arc is constructed from.
   * @param theta  The angle, in radiuans, subtended by this arc. Positive
   *  values refer to counterclockwise rotation, while negative values refer to
   *  clockwise rotation. Values greater in magnitude than 2pi represent arcs
   *  that loop around the circle multiple times before terminating.
   * @param off  The offset, relative to the center of the circle, of the start
   *  point of this path.
   */
  protected CircularArc(Point center, double theta, Point off) {
    this.center = center;
    this.theta = theta;
    this.off = off;
    this.radius = off.getNorm();
  }

  @Override
  public double getLength() {
    return radius * theta;
  }

  @Override
  public Point getPos(double distance) {
    return off.rotateBy(new Rotation2d(distance / radius)).plus(center);
  }

  @Override
  public Rotation2d getRotation(double distance) {
    double phi = Math.copySign(Math.PI / 2 + distance / radius, theta);
    return new Rotation2d(phi).plus(off.getAngle());
  }

  @Override
  public Rotation2d getAngularVelocity(double distance) {
    return new Rotation2d(Math.signum(theta) / radius);
  }

  @Override
  public CircularArc translateBy(Translation2d offset) {
    return new CircularArc(center.plus(offset), getStart().plus(offset), theta);
  }

  @Override
  public CircularArc rotateBy(Rotation2d rotation) {
    return new CircularArc(center.rotateBy(rotation), getStart().rotateBy(rotation), theta);
  }

  /**
   * Determines whether this arc passes through a ray.
   * 
   * @param ray  the direction fo the ray.
   * @return True, if a ray from the center of the circle of this arc through
   *  the specified point intersects this arc, or false, otherwise.
   */
  protected boolean includesRay(Point ray) {
    if(theta > 2*Math.PI || theta < 2*Math.PI) return true;
    Rotation2d phi = Point.getAngle(off, center, ray.minus(center));
    Rotation2d acuteTheta = new Rotation2d(theta);
    if(phi.equals(acuteTheta)) return true;
    return (phi.getCos() > acuteTheta.getCos() &&
        phi.getSin() * acuteTheta.getSin() > 0) ^
        acuteTheta.getRadians() == theta;
  }

  @Override
  public double getDistance(Point from) {
    if(includesRay(from)) return Math.abs(center.getDistance(from) - radius);
    else return Math.min(getStart().getDistance(from), getEnd().getDistance(from));
  }

  @Override
  public CircularArc reverse() {
    return new CircularArc(center, -theta, off.rotateBy(new Rotation2d(theta)));
  }

  /**
     * Gets the minimum distance between this path and another path.
     * 
     * <p>Implements {@link Path#getDistance(Path)} for the case where this path
     * is a CircularArc and the other path is a LinearSegment.
     * 
     * @param other  The path to find the distance to.
     * @return The minimum distance between any pair point along this path with
     *  any point on the other path.
     */
  public double getDistance(LinearSegment line) {
    Point nearest = line.getNearest(center);
    double minDist = center.getDistance(nearest);
    if(minDist >= radius){
      if(includesRay(nearest)) return minDist - radius;
      return Math.min(line.getDistance(getStart()), line.getDistance(getEnd()));
    }
    Point start = line.getStart(), end = line.getEnd();
    Point farthest = center.getDistance(start) > center.getDistance(end) ? start : end;
    double maxDist = center.getDistance(farthest);
    if(maxDist <= radius){
      if(includesRay(farthest)) return radius - maxDist;
      return Math.min(line.getDistance(getStart()), line.getDistance(getEnd()));
    }
    Rotation2d phi = new Rotation2d(Math.acos(minDist / radius));
    Point a = new Point(radius, nearest.getAngle().plus(phi)).plus(center);
    Point b = new Point(radius, nearest.getAngle().minus(phi)).plus(center);
    if(Point.getAngle(start, a, end).getCos() == -1 && includesRay(a))
      return 0;
    if(Point.getAngle(start, b, end).getCos() == -1 && includesRay(b))
      return 0;
    return Math.min(line.getDistance(getStart()), line.getDistance(getEnd()));
  }
  /**
     * Gets the minimum distance between this path and another path.
     * 
     * <p>Implements {@link Path#getDistance(Path)} for the case where this path
     * and the other path are both CircularArcs.
     * 
     * @param other  The path to find the distance to.
     * @return The minimum distance between any pair point along this path with
     *  any point on the other path.
     */
  public double getDistance(CircularArc other) {
    double dist = center.getDistance(other.center);
    if(dist >= radius + other.radius && includesRay(other.center) && other.includesRay(center))
      return dist - radius - other.radius;
    if(dist < radius + other.radius){
      Rotation2d phi = new Rotation2d(Math.acos(radius*radius/dist));
      Rotation2d angle = other.center.minus(center).getAngle();
      Point a = center.plus(new Point(radius, angle.plus(phi)));
      if(includesRay(a) && other.includesRay(a)) return 0;
      a = center.plus(new Point(radius, angle.minus(phi)));
      if(includesRay(a) && other.includesRay(a)) return 0;
    }
    return Math.min(Math.min(other.getDistance(getStart()), other.getDistance(getEnd())),
        Math.min(getDistance(other.getStart()), getDistance(other.getEnd())));
  }

  @Override
  public double getDistance(Path other) {
    if(other instanceof LinearSegment) return getDistance((LinearSegment) other);
    else if(other instanceof CircularArc) return getDistance((CircularArc) other);
    else return other.getDistance((CircularArc) this);
  }
}