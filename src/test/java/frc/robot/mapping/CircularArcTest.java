package frc.robot.mapping;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class CircularArcTest { 
    @Test
    public void getLengthTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        double length = arc.getLength();
        double trueLength = Math.sqrt(1) * 3.14;
        assertEquals(length, trueLength);
    }

    public void getPosTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        Point position = arc.getPos(2.0);
        Point truePosition = new Point(2,3);
        assertEquals(position, truePosition);
    }

    public void getRotationTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        Rotation2d rotation = arc.getRotation(1.0);
        Rotation2d trueRotation = new Rotation2d(Math.sqrt(2), Math.sqrt(2));
        assertEquals(rotation, trueRotation);

    }

    public void getAngularVelocityTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        Rotation2d angularVelocity = arc.getAngularVelocity(1.0);
        Rotation2d trueAngularVelocity = new Rotation2d(1, 1);
        assertEquals(angularVelocity, trueAngularVelocity);
    }

    public void translateByTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        CircularArc translate = arc.translateBy(new Translation2d(2, 3.14));
        CircularArc trueTranslate = new CircularArc(new Point(0,3), new Point(0,2), 0);
        assertEquals(translate, trueTranslate);
    }

    public void rotateByTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        CircularArc rotate = arc.rotateBy(new Rotation2d(2, 3.14));
        CircularArc trueRotate = new CircularArc(new Point(0,3), new Point(0,2), 0);
        assertEquals(rotate, trueRotate); 
    }

    public void reverseTest()
    {
        CircularArc arc = new CircularArc(new Point(0,0), 3.14, new Point(0,2));
        CircularArc reverse = arc.reverse();
        CircularArc trueReverse = new CircularArc(new Point(0,0), -3.14, new Point(0,2).rotateBy(new Rotation2d(3.14)));
        assertEquals(reverse, trueReverse);
    }
    public void getDistanceTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        double distance = arc.getDistance(new Point(0,2));
        double trueDistance = 2;
        assertEquals(distance, trueDistance);
    }

}
