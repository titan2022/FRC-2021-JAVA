// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.UsbCameraInfo;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private final UsbCamera camera;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    camera = CameraServer.getInstance().startAutomaticCapture();
  }

  public VisionSubsystem(int imageWidth, int imageHeight) {

    this();
    setResolution(imageWidth, imageHeight);

  }

  public VisionSubsystem(int imageWidth, int imageHeight, int brightness) {

    this(imageWidth, imageHeight);
    setBrightness(brightness);

  }

  public void setResolution(int imageWidth, int imageHeight) {
    camera.setResolution(imageWidth, imageHeight);
  }

  public void setBrightness(int brightness) {
    camera.setBrightness(brightness);
  }

  public UsbCamera getCamera() {
    return camera;
  }

  public void stopCamera() {
    camera.close();
  }

  public UsbCameraInfo getCameraInfo() {
    return camera.getInfo();
  }

  public boolean isConnected() {
    return camera.isConnected();
  }

  public boolean isEnabled() {
    return camera.isEnabled();
  }

  public boolean isValid() {
    return camera.isValid();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
