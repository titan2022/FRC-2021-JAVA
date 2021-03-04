/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.path.rrt;

import java.util.Random;
import java.util.ArrayList;

public class RRTF
{
    private final int maxNodes = 100;
    private int radius = 10;
    private final double[] start = new double[]{0.0,0.0};
    private final double[] end = new double[]{600.0,600.0};
    private ArrayList<double[]> path;
    public RRTF()
    {
        path = new ArrayList<double[]>();
        makePath(start);
    }
    public void makePath(double[] p1)
    {   
        double[] rNode = randomNode(p1);
        if (distance(p1, end) > distance(rNode, end))
        {
            makePath(p1);
        } 
        else
        {
            path.add(rNode);
        }
    }
        
    public double[] randomNode(double[] p1)
    {
        Random random = new Random();
        double coefficient = random.nextDouble();
        double t = 2 * Math.PI * coefficient;
        double x = radius * Math.cos(t);
        double y = radius * Math.sin(t);
        p1 = new double[]{x,y};
        return p1;
    }
    public double distance(double[] p1, double[] p2)
    {
        return (p2[0]-p1[0]) * (p2[0]-p1[0])+ (p2[1] - p1[1]) * (p2[1]-p1[1]);
    }

}
