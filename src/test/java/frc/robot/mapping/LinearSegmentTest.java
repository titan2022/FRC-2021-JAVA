package frc.robot.mapping;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * @Author Abhi Vinnakota
 */
public class LinearSegmentTest {
    
    @Test
    public void getLengthTest()
    {
        LinearSegment line = new LinearSegment(new Point(0,0), new Point(1,0));
        double length = line.getLength();
        double trueLength = Math.sqrt(0*0 + 1*1);
        assertEquals(length, trueLength);
    }

    @Test
    public void getPosTest()
    {
        LinearSegment line = new LinearSegment(new Point(0,0), new Point(1,0));
        Point position = line.getPos(2.0);
        Point truePosition = new Point(2,0);
        assertEquals(position, truePosition);
    }

    @Test
    public void getRotationTest()
    {
        LinearSegment line = new LinearSegment(new Point(0,0), new Point(1,0));
        Rotation2d rotation = line.getRotation(1.0);
        Rotation2d trueRotation = new Rotation2d(1,0);
        assertEquals(rotation,trueRotation);
    }

    @Test
    public void getAngularVelocityTest() 
    {
        LinearSegment line = new LinearSegment(new Point(0,0), new Point(1,0));
        Rotation2d angularVelocity = line.getAngularVelocity(1.0);
        Rotation2d trueAngularVelocity = new Rotation2d(0);
        assertEquals(angularVelocity, trueAngularVelocity);
    }

    @Test
    public void translateByTest() 
    {
        LinearSegment seg3 = new LinearSegment( new Point(0, 0), new Point(1, 0));
        LinearSegment translate = seg3.translateBy(new Translation2d(1,0));
        LinearSegment trueTranslate = new LinearSegment( new Point(0+1, 0), new Point(1+1, 0));
        assertEquals(translate, trueTranslate);
    }

    @Test
    public void rotateByTest() 
    {
        LinearSegment seg4 = new LinearSegment( new Point(1, 0), new Point(1, 1));
        LinearSegment rotate = seg4.rotateBy(new Rotation2d(30));
        LinearSegment trueRotate = new LinearSegment( new Point(0,0), new Point(.5,.5*Math.sqrt(3)));
        assertEquals(rotate, trueRotate);
    }

    @Test
    public void getDistanceTest() 
    {
        LinearSegment line = new LinearSegment(new Point(0,0), new Point(1,0));
        double distance = line.getDistance(new Point(-1,0));
        double trueDistance = 1.0;
        assertEquals(distance, trueDistance);
    }

    @Test
    public void getNearestTest() 
    {
        LinearSegment line = new LinearSegment(new Point(0,0), new Point(1,0));
        Point nearest = line.getNearest(new Point(-1,0));
        Point trueNearest = new Point(0,0);
        assertEquals(nearest, trueNearest);
    }

    @Test
    public void intersectsTest() 
    {
        LinearSegment line = new LinearSegment(new Point(0,0), new Point(1,0));
        boolean intersection = line.intersects(new LinearSegment(new Point(-2,0), new Point(-1,0)));
        boolean trueIntersection = false;
        assertEquals(intersection, trueIntersection);
    }

    @Test
    public void getDistanceTest1() 
    {
        LinearSegment line = new LinearSegment(new Point(0,0), new Point(1,0));
        double distance = line.getDistance(new LinearSegment(new Point(-2,0), new Point(-1,0)));
        double trueDistance = 1.0;
        assertEquals(distance, trueDistance);
    }

    @Test
    public void reverseTest()
    {
        Point start = new Point(0,0);
        Point end = new Point(1,0);
        LinearSegment line = new LinearSegment(start, end);
        LinearSegment reverse = line.reverse();
        LinearSegment trueReverse = new LinearSegment(end, start);
        assertEquals(reverse, trueReverse);
    }

    @Test
    public void getDistanceTest2()
    {
        LinearSegment line = new LinearSegment(new Point(0,0), new Point(1,0));
        double distance = line.getDistance(new LinearSegment(new Point(-2,0), new Point(-1,0)));
        double trueDistance = 1.0;
        assertEquals(distance, trueDistance);

    }
}
