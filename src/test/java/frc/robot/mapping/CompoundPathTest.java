package frc.robot.mapping;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * @Author Abhi Vinnakota
 */
public class CompoundPathTest {
    @Test
    public void getLengthTest() 
    {
        CompoundPath path = new CompoundPath(new LinearSegment(new Point(0,0), new Point(1,0)));
        double length = path.getLength();
        double trueLength = 1.0;
        assertEquals(length, trueLength);
    }
    @Test
    public void getPosTest() 
    {
        CompoundPath path = new CompoundPath(new LinearSegment(new Point(0,0), new Point(1,0)));
        Point position = path.getPos(1.0);
        Point truePosition = new Point(1,0);
        assertEquals(position, truePosition);
    }
    @Test
    public void getEndTest() 
    {
        CompoundPath path = new CompoundPath(new LinearSegment(new Point(0,0), new Point(1,0)));
        Point end = path.getEnd();
        Point trueEnd = new Point(1,0);
        assertEquals(end, trueEnd);
    }
    @Test
    public void getRotationTest() 
    {
        CompoundPath path = new CompoundPath(new LinearSegment(new Point(0,0), new Point(1,0)));
        Rotation2d rotation = path.getRotation(1);
        Rotation2d trueRotation = new Rotation2d(0,0);
        assertEquals(rotation, trueRotation);

    }
    @Test
    public void getAngularVelocityTest() 
    {
        CompoundPath path = new CompoundPath(new LinearSegment(new Point(0,0), new Point(1,0)));
        Rotation2d angularVelocity = path.getAngularVelocity(1);
        Rotation2d trueAngularVelocity = new Rotation2d(0,0);
        assertEquals(angularVelocity, trueAngularVelocity);
    }
    @Test
    public void translateByTest() 
    {
        CompoundPath path = new CompoundPath(new LinearSegment(new Point(0,0), new Point(1,0)));
        CompoundPath translation = path.translateBy(new Translation2d(0,1));
        CompoundPath trueTranslation = new CompoundPath(new LinearSegment(new Point(0,1), new Point(1,1)));
        assertEquals(translation, trueTranslation);

    }
    @Test
    public void rotateByTest() 
    {
        CompoundPath path = new CompoundPath(new LinearSegment(new Point(0,0), new Point(1,0)));
        CompoundPath rotation = path.rotateBy(new Rotation2d(0,1));
        CompoundPath trueRotation = new CompoundPath(new LinearSegment(new Point(0,1), new Point(1,1)));
        assertEquals(rotation, trueRotation);
    }
    @Test
    public void reverseTest() 
    {
        CompoundPath path = new CompoundPath(new LinearSegment(new Point(0,0), new Point(1,0)));
        CompoundPath reverse = path.reverse();
        CompoundPath trueReverse = new CompoundPath(new LinearSegment(new Point(1,0), new Point(0,0)));
        assertEquals(reverse, trueReverse);
    }
    @Test
    public void getDistanceTest() 
    {
        CompoundPath path = new CompoundPath(new LinearSegment(new Point(0,0), new Point(1,0)));
        double distance = path.getDistance(new Point(-1,0));
        double trueDistance = 1.0;
        assertEquals(distance, trueDistance);
    }
}