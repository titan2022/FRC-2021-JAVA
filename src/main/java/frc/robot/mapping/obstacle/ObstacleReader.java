// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mapping.obstacle;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

/** Add your docs here. */
public class ObstacleReader {

    public static String OBSTACLE_FILE_NAME = "obstacles.csv";

    public static void read() throws FileNotFoundException, IOException {

        BufferedReader reader = new BufferedReader(new FileReader(OBSTACLE_FILE_NAME));

        reader.close();

    }
    
}
