package frc.robot.mapping;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * @Author Abhi Vinnakota
 */
public class CircularArcTest { 
    @Test
    public void getLengthTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        double length = arc.getLength();
        double trueLength = Math.sqrt(1) * 3.14;
        assertEquals(length, trueLength);
    }
    @Test
    public void getPosTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), Math.PI*2);
        Point position = arc.getPos(Math.PI);
        Point truePosition = new Point(0,2);
        assertEquals(position, truePosition);
    }
    @Test
    public void getRotationTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), Math.PI*2);
        Rotation2d rotation = arc.getRotation(1.0);
        Rotation2d trueRotation = new Rotation2d(1);
        assertEquals(rotation, trueRotation);

    }
    @Test
    public void getAngularVelocityTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        Rotation2d angularVelocity = arc.getAngularVelocity(1.0);
        Rotation2d trueAngularVelocity = new Rotation2d(1);
        assertEquals(angularVelocity, trueAngularVelocity);
    }
    @Test
    public void translateByTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        CircularArc translate = arc.translateBy(new Translation2d(2, 3.14));
        CircularArc trueTranslate = new CircularArc(new Point(0,3), new Point(2,3.14), 0);
        assertEquals(translate.getPos(0), trueTranslate.getPos(0));
    }
    @Test
    public void rotateByTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        CircularArc rotate = arc.rotateBy(new Rotation2d(2, 3.14));
        CircularArc trueRotate = new CircularArc(new Point(0,3), new Point(0,0), 0);
        assertEquals(rotate.getPos(0), trueRotate.getPos(0)); 
    }
    @Test
    public void reverseTest()
    {
        CircularArc arc = new CircularArc(new Point(0,0), 3.14, new Point(0,2));
        CircularArc reverse = arc.reverse();
        CircularArc trueReverse = new CircularArc(new Point(0,0), -3.14, new Point(0,2).rotateBy(new Rotation2d(3.14)));
        assertEquals(reverse.getPos(0), trueReverse.getPos(0));
    }
    @Test
    public void getDistanceTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        double distance = arc.getDistance(new Point(0,2));
        double trueDistance = 0;
        assertEquals(distance, trueDistance);
    }
}
