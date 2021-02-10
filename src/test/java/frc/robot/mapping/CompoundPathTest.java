package frc.robot.mapping;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class CompoundPathTest {
    @Test
    public void getLengthTest() 
    {
        CompoundPath path = new CompoundPath();
        double length = path.getLength();
        double trueLength = 2.0;
        assertEquals(length, trueLength);
    }

    public void getPosTest() 
    {
        CompoundPath path = new CompoundPath();
        Point length = path.getPos(1.0);
        Point trueLength = new Point(1,1);
        assertEquals(length, trueLength);
    }

    public void getEndTest() 
    {
        CompoundPath path = new CompoundPath();
        Point end = path.getEnd();
        Point trueEnd = new Point(0,0);
        assertEquals(end, trueEnd);
    }

    public void getRotationTest() 
    {
        CompoundPath path = new CompoundPath();
        Rotation2d rotation = path.getRotation(0);
        Rotation2d trueRotation = new Rotation2d(0,0);
        assertEquals(rotation, trueRotation);

    }

    public void getAngularVelocityTest() 
    {
        CompoundPath path = new CompoundPath();
        Rotation2d angularVelocity = path.getAngularVelocity(0);
        Rotation2d trueAngularVelocity = new Rotation2d(0,0);
        assertEquals(angularVelocity, trueAngularVelocity);
    }

    public void translateByTest() 
    {
        CompoundPath path = new CompoundPath();
        CompoundPath translation = path.translateBy(new Translation2d());
        CompoundPath trueTranslation = new CompoundPath();
        assertEquals(translation, trueTranslation);

    }

    public void rotateByTest() 
    {
        CompoundPath path = new CompoundPath();
        CompoundPath rotation = path.rotateBy(new Rotation2d());
        CompoundPath trueRotation = new CompoundPath();
        assertEquals(rotation, trueRotation);
    }

    public void reverseTest() 
    {
        CompoundPath path = new CompoundPath();
        CompoundPath reverse = path.reverse();
        CompoundPath trueReverse = new CompoundPath();
        assertEquals(reverse, trueReverse);
    }

    public void getDistanceTest() 
    {
        CompoundPath path = new CompoundPath();
        double distance = path.getDistance(new Point(0,0));
        double trueDistance = 0.0;
        assertEquals(distance, trueDistance);
    }

    public void getDistanceTest1() 
    {
        CompoundPath path = new CompoundPath();
        double distance = path.getDistance(new Path());
        double trueDistance = 0.0;
        assertEquals(distance, trueDistance);
    }
}