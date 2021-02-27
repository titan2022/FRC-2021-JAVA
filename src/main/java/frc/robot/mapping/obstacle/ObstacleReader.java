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

/**
 * ObstacleReader reads from csv and puts into ObstacleMap.
 */
public class ObstacleReader {

    public final static String OBSTACLE_FILE_NAME = "obstacles.csv";

    /**
     * Reads from csv into ObstacleMap.
     * @return Obstacle map.
     * @throws FileNotFoundException
     * @throws IOException
     */
    public static ObstacleMap read() throws FileNotFoundException, IOException {

        Path csvPath = Filesystem.getDeployDirectory().toPath().resolve(OBSTACLE_FILE_NAME);
        BufferedReader reader = new BufferedReader(new FileReader(new File(csvPath.toString())));
        
        ObstacleMap map = new ObstacleMap();
        ArrayList<Point> points = new ArrayList<Point>();

        String line;
        Point[] pointsArray;
        String[] coordStrings;

        while ((line = reader.readLine()) != null) {

            if (line == ",") {

                pointsArray = points.toArray(new Point[points.size()]);
                map.addObstacle(new Polygon(pointsArray));
                points.clear();

            } else {

                coordStrings = line.split(",");
                points.add(new Point(parseDouble(coordStrings[0]), parseDouble(coordStrings[1])));

            }

        }

        reader.close();
        return map;

    }

    /**
     * Reads from csv into ObstacleMap without exceptions
     * @return Obstacle map.
     */
    public static ObstacleMap readWithoutExceptions() {

        try {

            return read();

        } catch (FileNotFoundException fileNotFoundException) {

            System.out.println("File not found (" + OBSTACLE_FILE_NAME + ").");
            return new ObstacleMap();

        } catch (IOException ioException) {

            System.out.println("I/O exception");
            return new ObstacleMap();

        }

    }

}
