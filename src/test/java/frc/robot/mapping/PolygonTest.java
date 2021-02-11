package frc.robot.mapping;


import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import static org.junit.jupiter.api.Assertions.assertEquals;
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
        Path boundary = poly.getBoundary(2.0);
        Path[] segTrue = new Path[8];
        for(int i=0; i<4; i++){
            segTrue[i*2] = new CircularArc(new Point(0,0), poly.verts[i],  poly.verts[i].plus(new Point(2,0)));
            segTrue[i*2+1] = new LinearSegment(new Point(2,2),new Point (2,0));
        }
        Path boundTrue = new CompoundPath(new Path[1]);
        assertEquals(boundary, boundTrue);
    }
    @Test
    public void getTangentsTest() {
        
    }
    @Test
    public void rotateByTest() {
        
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
}