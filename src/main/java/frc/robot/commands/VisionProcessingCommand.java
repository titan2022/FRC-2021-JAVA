// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.vision.VisionRunner.Listener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.ContourPipeline;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class VisionProcessingCommand extends CommandBase {

  private final UsbCamera camera;
  private final VisionThread thread;
  private final Object imageLocker = new Object();
  private double centerX = 0.0, centerY = 0.0;

  /** Creates a new VisionProcessingCommand. */
  public VisionProcessingCommand(VisionSubsystem vision, ContourPipeline pipeline) {
    // Use addRequirements() here to declare subsystem dependencies.
    camera = vision.getCamera();
    thread = new VisionThread(camera, pipeline, makeListener());
    thread.start();
  }

  private Listener<ContourPipeline> makeListener() {

    return pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized (imageLocker) {
            centerX = r.x + (r.width / 2);
            centerY = r.y + (r.height / 2);
            SmartDashboard.putNumber("center x", centerX);
            SmartDashboard.putNumber("center y", centerY);
        }
      }
    };
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
