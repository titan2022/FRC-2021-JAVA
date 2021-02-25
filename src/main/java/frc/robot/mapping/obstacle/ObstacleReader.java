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
import frc.robot.mapping.Polygon;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Point;
import java.util.ArrayList;
import static java.lang.Double.parseDouble;

import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class ObstacleReader {

    public static String OBSTACLE_FILE_NAME = "obstacles.csv";

    public static ObstacleMap read() throws FileNotFoundException, IOException {

        ObstacleMap map = new ObstacleMap();
        Path csvPath = Filesystem.getDeployDirectory().toPath().resolve(OBSTACLE_FILE_NAME);
        BufferedReader reader = new BufferedReader(new FileReader(new File(csvPath.toString())));
        String line;
        ArrayList<Point> points = new ArrayList<Point>();

        while ((line = reader.readLine()) != null) {

            if (line == ",") {

                Point[] pointsArray = points.toArray(new Point[points.size()]);
                map.addObstacle(new Polygon(pointsArray));

            } else {

                String[] coordStrings = line.split(",");
                points.add(new Point(parseDouble(coordStrings[0]), parseDouble(coordStrings[1])));

            }

        }

        reader.close();
        return map;

    }

}
