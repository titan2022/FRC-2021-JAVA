package frc.robot.mapping;


import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
public class PolygonTest {
    @Test
    public void CompareTest() {
        
        int trueComp = 0;
        
        Point x = new Point (0,1);
        Polygon poly = new Polygon(new Point(-2,-2), new Point (-2,2), new Point (2,2), new Point(2,-2)); //index out of bounds at 40?
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

    }
    @Test
    public void getBoundaryTest() {
        
    }
    @Test
    public void getTangentsTest() {
        
    }
    @Test
    public void rotateByTest() {
        
    }
    @Test
    public void translateByTest() {
        
    }
}