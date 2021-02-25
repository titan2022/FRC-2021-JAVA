// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mapping.obstacle;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class ObstacleReader {

    public static String OBSTACLE_FILE_NAME = "obstacles.csv";

    public static void read() throws FileNotFoundException, IOException {

        Path csvPath = Filesystem.getDeployDirectory().toPath().resolve(OBSTACLE_FILE_NAME);
        BufferedReader reader = new BufferedReader(new FileReader(new File(csvPath.toString())));
        String line;

        while ((line = reader.readLine()) != null) {

            System.out.println(line);

        }

        reader.close();

    }
    
}
