package frc.robot.mapping;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class LinearSegmentTest {
    @Test
    public void getLengthTest()
    {
        LinearSegment seg = new LinearSegment( new Point(1, 0), new Point(1, 2));
        double length = seg.getLength();
        double trueLength = Math.sqrt((1-1) * (1-1) + 2 * 2);
        assertEquals(length, trueLength);
    }

    public void getPosTest()
    {
        Point one = new Point (0.5, 0.5*Math.sqrt(3));
        Point two = new Point (0,0);
        LinearSegment seg2 = new LinearSegment( two, one);
        Point truPosition = new Point(5, one.minus(two).getAngle());
        Point position = seg2.getPos(5.0);
        assertEquals(position, truPosition);
    }
}
