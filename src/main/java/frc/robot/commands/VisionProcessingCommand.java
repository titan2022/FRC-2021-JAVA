// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.vision.VisionRunner.Listener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motion.control.CustomKalmanFilter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.ContourPipeline;

import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class VisionProcessingCommand extends CommandBase {

  private static final double STATE_STD_DEV = 1; // GRIP units
  private static final double MEAS_STD_DEV = 10; // GRIP units

  private final UsbCamera camera;
  private final VisionThread thread;
  private final Object imageLocker = new Object();
  private final CustomKalmanFilter filter;
  private double centerX, centerY;

  /** Creates a new VisionProcessingCommand. */
  public VisionProcessingCommand(VisionSubsystem vision, ContourPipeline pipeline) {
    // Use addRequirements() here to declare subsystem dependencies.
    camera = vision.getCamera();
    thread = new VisionThread(camera, pipeline, makeListener());
    thread.start();
    filter = new CustomKalmanFilter(new SimpleMatrix(2, 1), SimpleMatrix.identity(2),
        SimpleMatrix.identity(2).scale(Math.pow(STATE_STD_DEV, 2)),
        SimpleMatrix.identity(2).scale(Math.pow(MEAS_STD_DEV, 2)), SimpleMatrix.identity(2), SimpleMatrix.identity(2),
        SimpleMatrix.identity(2));
  }

  private Listener<ContourPipeline> makeListener() {

    return pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized (imageLocker) {
          centerX = r.x + (r.width / 2);
          centerY = r.y + (r.height / 2);
          SmartDashboard.putNumber("unfilt center x", centerX);
          SmartDashboard.putNumber("unfilt center y", centerY);
          filter.updateFilter(new SimpleMatrix(new double[][] { { centerX }, { centerY } }));
          SmartDashboard.putNumber("filt center x", filter.getState().get(0, 0));
          SmartDashboard.putNumber("filt center y", filter.getState().get(1, 0));
        }
      }
    };
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    thread.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
