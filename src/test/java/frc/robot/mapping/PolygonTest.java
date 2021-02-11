package frc.robot.mapping;


import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Set;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
public class PolygonTest {
    @Test
    public void CompareTest() {
        
        int trueComp = 0;
        
        Point x = new Point (0,1);
        Polygon poly = new Polygon(new Point(-2,-2), new Point (-2,2), new Point (2,2), new Point(2,-2));
        int comp = poly.compare(new Point(0,1), new Point(0,-1));
        if (x.getAngle().getRadians()-(new Point(0,-1)).getAngle().getRadians() > 0.0) {
            trueComp = 1;
        }
        else if (x.getAngle().getRadians()-(new Point(0,-1)).getAngle().getRadians() < 0) {
            trueComp = -1;
        }
        
        assertEquals(1, trueComp);
    }
    @Test
    public void getPerimeterTest() {
        double truePerim = 16;
        Polygon poly = new Polygon(new Point(-2,-2), new Point (-2,2), new Point (2,2), new Point(2,-2));
        double perim = poly.getPerimeter();
        assertEquals(truePerim, perim);
    }
    @Test
    public void getBoundaryTest() {
        Polygon poly = new Polygon(new Point(-1,-1), new Point (-1,1), new Point (1,1), new Point(1,-1));
        Path boundary = poly.getBoundary(0);
        double len = boundary.getLength();
        double lenTrue = 8;
        assertEquals(len, lenTrue);
    }
    @Test
    public void getTangentsTest() {
        Polygon poly = new Polygon(new Point(-1,-1), new Point (-1,0), new Point (0,0), new Point(0,-1));
        Polygon poly2 = new Polygon(new Point(0,0), new Point (0,1), new Point (1,0), new Point(1,1));
        Iterable<LinearSegment> tan = poly.getTangents(poly2);
        LinearSegment tan1 = new LinearSegment(new Point(-1,0), new Point(0,1));
        LinearSegment tan2 = new LinearSegment(new Point(0,-1), new Point(0,1));
        List<LinearSegment> truTan = new ArrayList<LinearSegment>(2);
        truTan.add(tan1);
        truTan.add(tan2);
        assertEquals(tan, truTan);
    }
    @Test
    public void rotateByTest() {
        Polygon poly = new Polygon(new Point(-1,-1), new Point (-1,1), new Point (1,1), new Point(1,-1));
        Polygon rotation = poly.rotateBy(new Rotation2d(0,1));
        Polygon truRotation = new Polygon(new Point(1,-1), new Point(-1,-1), new Point (-1,1), new Point (1,1));
        assertEquals(rotation.verts, truRotation.verts); 
    }
    @Test
    public void translateByTest() {
        Polygon poly = new Polygon(new Point(-2,-2), new Point (-2,2), new Point (2,2), new Point(2,-2));
        Polygon transl = poly.translateBy(new Translation2d (1,1));
        Point[] transl2 = transl.verts;
        Polygon trueTransl = new Polygon(new Point(2+1,-2+1), new Point (2+1,2+1), new Point (-2+1,2+1), new Point(-2+1,-2+1));
        Point[] trueTransl2 = trueTransl.verts;
        
        assertEquals(transl2[0], trueTransl2[0]);
        assertEquals(transl2[1], trueTransl2[1]);
        assertEquals(transl2[2], trueTransl2[2]);
        assertEquals(transl2[3], trueTransl2[3]);
    }
    @Test
    public void edgePathTest() {
        Polygon poly = new Polygon(new Point(-1,-1), new Point (-1,1), new Point (1,1), new Point(1,-1));
         
    }
}