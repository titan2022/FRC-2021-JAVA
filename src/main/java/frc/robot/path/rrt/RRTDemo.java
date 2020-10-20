/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.path.rrt;

import edu.wpi.first.wpilibj.RobotBase;
import javax.swing.*; 
import java.awt.*; 
    
public class RRTDemo extends JFrame{

    private final int rows = 10;
    private final int columns = 10;
    private final int[][] grid = {{1,1,1,2,1,1,1,1,1,1}, {1,1,1,2,1,1,1,1,1,1}, {1,1,1,2,1,1,1,1,1,1}, {1,1,1,2,1,1,1,1,1,1}, {1,1,1,2,1,1,1,1,1,1}, {1,1,1,2,1,1,1,1,1,1}, {1,1,1,2,1,1,1,1,1,1}, {1,1,1,2,1,1,1,1,1,1}, {1,1,1,2,1,1,1,1,1,1}, {1,1,1,2,1,1,1,1,1,1}};
    ;
    public RRTDemo()
    {
        JFrame f = new JFrame();
        f = new JFrame("map");
        f.setSize(600, 600); 
        f.repaint();
        try { 
         // set look and feel 
        UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName()); 
        }   
        catch (Exception e) { 
            System.err.println(e.getMessage()); 
        }
        f.setVisible(true);
    }

  @Override
  public void paint(Graphics g){
      Graphics2D g2 = (Graphics2D) g;
      int width = getSize().width;
      int height = getSize().height;
  
      int rowHeight = height / (rows);
      int colWidth = width / (columns);
      for (int row = 0; row < rows; row++) {
        for (int col = 0; col < columns; col++) 
            {
            switch (this.grid[row][col]) 
                {
                case 1:
                    g.setColor(Color.WHITE);
                    break;
                case 2:
                    g.setColor(Color.BLACK);
                    break;
                }
            g.drawRect(col * colWidth, height - ((row + 1) * rowHeight), 60, 60);
            g.fillRect(col * colWidth, height - ((row + 1) * rowHeight), 60, 60);
             }
        }
    }
}


