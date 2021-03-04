/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.path.rrt;

import java.util.ArrayList;
 
/**
 * Add your docs here.
 */
public class NodeF
{
    public final int x_dim = 600;
    public final int y_dim = 600;
    public NodeF(double x, double y)    {
        
    }
    public double distance(double[] p1, double[] p2)
    {
        return (p2[0]-p1[0]) * (p2[0]-p1[0])+ (p2[1] - p1[1]) * (p2[1]-p1[1]);
    }
    
    public boolean collision(double[] p1, double[] p2, double radius)
    {
        double distance = distance(p1,p2);
        if (distance > radius)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

}
