package frc.robot.mapping;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Translation2d;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import static org.junit.jupiter.api.Assertions.assertEquals;
public class PointTest {
    @Test 
    public void getAngleTest() {
        Point pt = new Point (.5,.5*Math.sqrt(3));
        Rotation2d tim = pt.getAngle();
        Rotation2d timTrue = new Rotation2d(Math.PI/3);
        assertEquals(tim, timTrue);
    } 
    @Test
    public void unaryMinusTest() {
        Point pt = new Point (1,5);
        Point tim = pt.unaryMinus();
        Point timTrue = new Point (-1, -5);
        assertEquals(tim, timTrue);
    }
    @Test
    public void rotateByTest() {
        Point pt = new Point (0,1);
        Point tim = pt.rotateBy(new Rotation2d(.5, .5*Math.sqrt(3)));
        Point timTrue = new Point (-0.5*Math.sqrt(3),.5);
        assertEquals(tim, timTrue);
    }
    @Test 
    public void plusTest() {
        Point pt = new Point (1,1);
        Point tim = pt.plus(new Translation2d (2.0, 3.0));
        Point timTrue = new Point (1+2.0, 1+3.0);
        assertEquals(tim, timTrue);
    }
    @Test 
    public void minusTest() {
        Point pt = new Point (1,1);
        Point tim = pt.minus(new Translation2d (2.0, 3.0));
        Point timTrue = new Point (1-2.0, 1-3.0);
        assertEquals(tim, timTrue);
    }
    @Test
    public void timesTest() {
        Point pt = new Point (1,1);
        Point tim = pt.times(5.0);
        Point timTrue = new Point (1*5.0, 1*5.0);
        assertEquals(tim, timTrue);
    }
    @Test
    public void divTest() {
        Point pt = new Point (1,1);
        Point tim = pt.div(5.0);
        Point timTrue = new Point (1/5.0, 1/5.0);
        assertEquals(tim, timTrue);
    }
    @Test
    public void toStringTest() {
        Point pt = new Point (1.11,1.22);
        String str = pt.toString();
        String trueStr = "Point(1.11, 1.22)";
        assertEquals(str, trueStr);
    }
}