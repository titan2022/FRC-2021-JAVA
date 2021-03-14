// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;

import edu.wpi.first.vision.VisionPipeline;

/** Add your docs here. */
public interface ContourPipeline extends VisionPipeline {

    public ArrayList<MatOfPoint> findContoursOutput();
    public ArrayList<MatOfPoint> filterContoursOutput();

}
