package frc.robot.mapping;


import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
public class ObstacleMapTest {
    @Test
    public void addObstacleTest() 
    {
        CircularArc arc = new CircularArc(new Point(0,1), new Point(0,0), 3.14);
        double length = arc.getLength();
        double trueLength = Math.sqrt(1) * 3.14;
        assertEquals(length, trueLength);
    }

    public void removeObstacleTest()
    {

    }

    public void getObstaclesTest()
    {

    }

    public void isClearTest()
    {

    }
    
    public void onAdditionTest()
    {

    }

    public void onRemovalTest()
    {
        
    }
}