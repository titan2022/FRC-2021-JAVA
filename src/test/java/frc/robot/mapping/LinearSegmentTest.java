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
}
