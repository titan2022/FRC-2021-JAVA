package frc.robot.mapping;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class LinearSegmentTest {
    @Test
    
    public void translateByTest() {
        LinearSegment seg3 = new LinearSegment( new Point(1, 0), new Point(1, 1));
        LinearSegment translate = seg3.translateBy(new Translation2d(4,5));
        LinearSegment trueTranslate = new LinearSegment( new Point(1+4, 0+5), new Point(1+4, 1+5));
        assertEquals(translate, trueTranslate);
    }
    public void rotateByTest() {
        LinearSegment seg4 = new LinearSegment( new Point(1, 0), new Point(1, 1));
        LinearSegment rotate = seg4.rotateBy(new Rotation2d(30));
        LinearSegment trueRotate = new LinearSegment( new Point(0,0), new Point(.5,.5*Math.sqrt(3)));
        assertEquals(rotate, trueRotate);
    }

    public void getRotationTest()
    {

    }

    public void getAngularVelocityTest() {

    }

    public void translateByTest() {

    }

    public void rotateByTest() {

    }

    public void getDistanceTest() {

    }

    public void getNearestTest() {

    }

    public void intersectsTest() {

    }

    public void getDistanceTest1() {

    }

    public void reverseTest()
    {
        
    }
}
