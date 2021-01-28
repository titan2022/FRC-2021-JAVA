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
    if(Point.getAngle(verts[0], interior, verts[verts.length-1]).getSin() > 0){
      for(int i=0; i<verts.length/2; i++){
        Point tmp = verts[i];
        verts[i] = verts[verts.length-i-1];
        verts[verts.length-i-1] = tmp;
      }
    }
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
    return vertex.plus(new Point(radius, new Rotation2d(theta).plus(from.minus(vertex).getAngle())));
  }

  @Override
  public Set<Point> getEndpoints(Point source, double radius) {
    Point argmin = null, argmax = null;
    double min = 360, max = -360.;
    for(int i = 0; i < verts.length; i++){
      Rotation2d phi0 = source.minus(verts[i]).getAngle();
      Rotation2d phi = new Rotation2d(Math.acos(Math.min(radius / verts[i].getDistance(source), 1)));
      Point tangent1 = verts[i].plus(new Point(radius, phi0.plus(phi)));
      Point tangent2 = verts[i].plus(new Point(radius, phi0.minus(phi)));
      double theta1 = Point.getAngle(interior, source, tangent1).getDegrees();
      double theta2 = Point.getAngle(interior, source, tangent2).getDegrees();
      if(theta1 < min){
        min = theta1;
        argmin = tangent1;
      }
      if(theta1 > max){
        max = theta1;
        argmax = tangent1;
      }
      if(theta2 < min){
        min = theta2;
        argmin = tangent2;
      }
      if(theta2 > max){
        max = theta2;
        argmax = tangent2;
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
        Point otherCurr = other.verts[j];
        Point otherNext = other.verts[(j+1)%other.verts.length];
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
  public Iterable<LinearSegment> getTangents(Obstacle other, double radius) {
    if(other instanceof Polygon)
      return getTangents((Polygon) other, radius);
    else
      return other.getTangents((Polygon) this, radius);
  }

  @Override
  public double getPerimeter(double radius) {
    return basePerimeter + radius * (verts.length-2) * Math.PI;
  }

  @Override
  public Path edgePath(Point a, Point b, double radius) {
    List<Path> suffix = new ArrayList<>();
    List<Path> prefix = new ArrayList<>();
    List<Path> complete = new ArrayList<>();  // accumulated path
    List<Path> running = suffix;
    Rotation2d theta = new Rotation2d(-Math.PI/2);
    Point seeking = null;
    Point alpha, beta;
    for(int i=0; i<verts.length; i++){
      Point vertex = verts[i];
      Point vNext = verts[i+1 == verts.length ? 0 : i+1];
      Point offset = new Point(radius, vNext.minus(vertex).getAngle().plus(theta));
      
      // Linear case
      alpha = vertex.plus(offset);
      beta = vNext.plus(offset);
      boolean aPres = Point.getAngle(beta, alpha, a).getCos() == 1;
      boolean bPres = Point.getAngle(beta, alpha, b).getCos() == 1;
      if(aPres && bPres){
        return new LinearSegment(a, b);
      }
      if(seeking == null){
        if(aPres){
          suffix.add(new LinearSegment(alpha, a));
          running = complete;
          alpha = a;
          seeking = b;
        }
        else if(bPres){
          suffix.add(new LinearSegment(alpha, b));
          running = complete;
          alpha = b;
          seeking = a;
        }
      }
      if(seeking == b && bPres){
        running.add(new LinearSegment(alpha, b));
        running = prefix;
        alpha = b;
        seeking = a;
      }
      if(seeking == a && aPres){
        running.add(new LinearSegment(alpha, a));
        running = prefix;
        alpha = a;
        seeking = b;
      }
      if(seeking != null){
        running.add(new LinearSegment(alpha, beta));
      }
      else{
        suffix.add(new LinearSegment(alpha, beta));
      }

      // Circular Case
      Point vNextNext = verts[i+2 >= verts.length ? i+2-verts.length : i+2];
      alpha = beta;
      beta = new Point(radius, beta.minus(vNext).getAngle().plus(
        new Rotation2d(Math.PI).minus(Point.getAngle(vNextNext, vNext, vertex))
      )).plus(vNext);
      //aPres = vNext.getDistance(a) <= radius + 0.001;
      //bPres = vNext.getDistance(b) <= radius + 0.001;
      aPres = Point.getAngle(alpha, vNext, a).getSin() >= 0 && Point.getAngle(a, vNext, beta).getSin() > 0;
      bPres = Point.getAngle(alpha, vNext, b).getSin() >= 0 && Point.getAngle(b, vNext, beta).getSin() > 0;
      if(aPres && bPres){
        return new CircularArc(a, vNext, b);
      }
      if(seeking == null){
        if(aPres){
          suffix.add(new CircularArc(alpha, vNext, a));
          running = complete;
          alpha = a;
          seeking = b;
        }
        else if(bPres){
          suffix.add(new CircularArc(alpha, vNext, b));
          running = complete;
          alpha = b;
          seeking = a;
        }
      }
      if(seeking == b && bPres){
        running.add(new CircularArc(alpha, vNext, b));
        running = prefix;
        alpha = b;
        seeking = a;
      }
      if(seeking == a && aPres){
        running.add(new CircularArc(alpha, vNext, a));
        running = prefix;
        alpha = a;
        seeking = b;
      }
      if(seeking != null){
        running.add(new CircularArc(alpha, vNext, beta));
      }
      else{
        suffix.add(new CircularArc(alpha, vNext, beta));
      }
    }
    double fullSum = 0, splitSum = 0;
    for(Path path : complete)
      fullSum += path.getLength();
    for(Path path : prefix)
      splitSum += path.getLength();
    for(Path path : suffix)
      splitSum += path.getLength();
    Path res;
    if(splitSum < fullSum){
      prefix.addAll(suffix);
      res = new CompoundPath(prefix.toArray(new Path[0]));
      if(seeking == a) res = res.reverse();
    }
    else{
      res = new CompoundPath(complete.toArray(new Path[0]));
      if(seeking == b) res = res.reverse();
    }
    return res;
  }

  public Path getBoundary(double radius) {
    Path[] segments = new Path[verts.length*2];
    Rotation2d theta = new Rotation2d(-Math.PI/2);
    Point prev = verts[0].plus(new Point(radius,
        verts[verts.length-1].minus(verts[0]).getAngle().minus(theta)));
    Point mid, offset, vNext;
    for(int i=0; i<verts.length; i++){
      vNext = verts[(i+1)%verts.length];
      offset = new Point(radius, vNext.minus(verts[i]).getAngle().plus(theta));
      mid = verts[i].plus(offset);
      segments[i*2] = new CircularArc(prev, verts[i], mid);
      prev = vNext.plus(offset);
      segments[i*2+1] = new LinearSegment(mid, prev);
    }
    return new CompoundPath(segments);
  } 
}
