package frc.robot.mapping;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class CircularArcTest { 
    @Test
    public void getLengthTest() {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        double length = arc.getLength();
        double trueLength = Math.sqrt(1) * 3.14;
        assertEquals(length, trueLength);
    }

    public void getPosTest() {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        Point position = arc.getPos(2.0);
        Point 
    }

    public void getRotationTest() {

    }

    public void getAngularVelocityTest() {

    }

    public void translateByTest() {

    }

    public void rotateByTest() {

    }

    @Override
    public void getDistanceTest() {

    }

    public void reverseTest() {

    }

    



}
