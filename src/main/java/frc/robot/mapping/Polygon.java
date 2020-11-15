package frc.robot.mapping;

import java.util.Set;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Polygon implements Obstacle {
  protected final Point[] verts;
  protected final Point interior;
  protected final double basePerimeter;

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

  protected Point getTangency(int vertIdx, Point from, double radius) {
    Point vertex = verts[vertIdx];
    double cos = radius / vertex.getDistance(from);
    if(cos > 1) return from;
    double theta = Math.copySign(Math.acos(cos), Point.getAngle(interior, vertex, from).getSin());
    return vertex.plus(new Point(radius, new Rotation2d(theta)));
  }

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

  public int compare(Point a, Point b) {
    return (int) Math.signum(
        a.minus(interior).getAngle().getRadians() -
        b.minus(interior).getAngle().getRadians()); 
  }

  public Polygon translateBy(Translation2d offset) {
    Point[] vertices = new Point[verts.length];
    int i = 0;
    for(Point vertex : verts) vertices[i++] = vertex.plus(offset);
    return new Polygon(vertices, interior.plus(offset), basePerimeter);
  }

  public Polygon rotateBy(Rotation2d rotation) {
    Point[] vertices = new Point[verts.length];
    int i = 0;
    for(Point vertex : verts) vertices[i++] = vertex.rotateBy(rotation);
    return new Polygon(vertices, interior.rotateBy(rotation), basePerimeter);
  }

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
}
