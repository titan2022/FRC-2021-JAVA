package frc.robot.mapping;

import java.util.Set;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * A convex polygon obstacle.
 */
public class Polygon implements Obstacle {
  /** The vertces of this polygon, in clockwise order. */
  protected final Point[] verts;
  /** A point in the intorerior of this polygon. */
  protected final Point interior;
  /** The length of the perimeter of this polygon, before obstacle growth. */
  protected final double basePerimeter;

  /**
   * Creates a polygon with the provided vertices.
   * 
   * @param vertices  The vertices of the polygon, in order.
   */
  public Polygon(Point... vertices) {
    verts = vertices;
    Point center = new Point(0, 0);
    double p0 = 0;
    for(int i=0; i<verts.length; i++){
      center = center.plus(verts[i]);
      p0 += verts[i].getDistance(verts[(i+1)%verts.length]);
    }
    interior = center.div(verts.length);
    basePerimeter = p0;
  }
  private Polygon(Point[] vertices, Point interior, double perimeter) {
    verts = vertices;
    basePerimeter = perimeter;
    this.interior = interior;
  }

  /**
   * Gets the point of tangency to a circle around a vertex.
   * 
   * @param vertIdx  The index of the vertex of this obstacle.
   * @param from  The endpoint of the tangent line.
   * @param radius  The radius around the vertex.
   * @return The point of tangency of a line passing through {@code from} to a
   *  circle of the given radius around the given vertex of this obstacle.
   */
  protected Point getTangency(int vertIdx, Point from, double radius) {
    Point vertex = verts[vertIdx];
    double cos = radius / vertex.getDistance(from);
    if(cos > 1) return from;
    double theta = Math.copySign(Math.acos(cos), Point.getAngle(interior, vertex, from).getSin());
    return vertex.plus(new Point(radius, new Rotation2d(theta)));
  }

  @Override
  public Set<Point> getEndpoints(Point source, double radius) {
    Point argmin = null, argmax = null, tangency;
    double min = 360, max = -360., theta;
    for(int i = 0; i < verts.length; i++){
      tangency = getTangency(i, source, radius);
      theta = Point.getAngle(interior, source, tangency).getDegrees();
      if(theta < min){
        min = theta;
        argmin = tangency;
      }
      if(theta > max){
        max = theta;
        argmax = tangency;
      }
    }
    Set<Point> endpoints = new LinkedHashSet<Point>(2, 1.0f);
    if(argmin != null) endpoints.add(argmin);
    if(argmax != null) endpoints.add(argmax);
    return endpoints;
  }

  @Override
  public int compare(Point a, Point b) {
    return (int) Math.signum(
        a.minus(interior).getAngle().getRadians() -
        b.minus(interior).getAngle().getRadians()); 
  }

  @Override
  public Polygon translateBy(Translation2d offset) {
    Point[] vertices = new Point[verts.length];
    int i = 0;
    for(Point vertex : verts) vertices[i++] = vertex.plus(offset);
    return new Polygon(vertices, interior.plus(offset), basePerimeter);
  }

  @Override
  public Polygon rotateBy(Rotation2d rotation) {
    Point[] vertices = new Point[verts.length];
    int i = 0;
    for(Point vertex : verts) vertices[i++] = vertex.rotateBy(rotation);
    return new Polygon(vertices, interior.rotateBy(rotation), basePerimeter);
  }

  /** Gets the tangent lines between this obstacle and another.
   * 
   * <p>Implements {@link Obstacle#getTangents(Obstacle, double)} for the case
   * where this obstacle and the other obstacle are both convex polygons.
   * 
   * @param other  The other obstacle to get tangent lines between.
   * @param radius  The obstacle growth radius to use.
   * @return An iterable over the tangent lines between this obstacle and the
   *  other specified obstacle. Each tangent is represented as a LinearSegment
   *  from the point of tangency with this obstacle to the point of tangency
   *  with the other obstacle.
   */
  public Iterable<LinearSegment> getTangents(Polygon other, double radius) {
    if(other == this) return new ArrayList<LinearSegment>(0);
    List<LinearSegment> res = new ArrayList<LinearSegment>(2);
    Rotation2d oca, ocb, cox, coy, phi;
    double thisSign, otherSign, magnitude;
    Point thisPrev = verts[verts.length-1], near, far;
    for(int i=0; i<verts.length; i++){
      Point thisCurr = verts[i];
      Point thisNext = verts[(i+1)%verts.length];
      Point otherPrev = other.verts[other.verts.length-1];
      for(int j=0; j<other.verts.length; j++){
        Point otherCurr = other.verts[i];
        Point otherNext = other.verts[(i+1)%other.verts.length];
        oca = Point.getAngle(otherCurr, thisCurr, thisPrev);
        ocb = Point.getAngle(otherCurr, thisCurr, thisNext);
        cox = Point.getAngle(thisCurr, otherCurr, otherPrev);
        coy = Point.getAngle(thisCurr, otherCurr, otherNext);
        if(oca.getSin() * ocb.getSin() < 0 || cox.getSin() * coy.getSin() < 0 ||
           oca.getCos() == 1 || ocb.getCos() == 1 || cox.getCos() == 1 || coy.getCos() == 1)
          continue;
        thisSign = -Point.getAngle(otherCurr, thisCurr, interior).getSin();
        otherSign = -Point.getAngle(thisCurr, otherCurr, other.interior).getSin();
        magnitude = thisSign * otherSign > 0 ? Math.acos(2*radius/thisCurr.getDistance(otherCurr)) : Math.PI/2;
        phi = new Rotation2d(Math.copySign(magnitude, thisSign)).plus(otherCurr.minus(thisCurr).getAngle());
        near = thisCurr.plus(new Point(radius, phi));
        phi = new Rotation2d(Math.copySign(magnitude, otherSign)).plus(thisCurr.minus(otherCurr).getAngle());
        far = otherCurr.plus(new Point(radius, phi));
        res.add(new LinearSegment(near, far));
      }
      thisPrev = thisCurr;
    }
    return res;
  }

  @Override
  public double getPerimeter(double radius) {
    return basePerimeter + radius * (verts.length-2) * Math.PI;
  }
}
