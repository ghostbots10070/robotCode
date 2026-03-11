//  Copyright (c) FIRST and other WPILib contributors.
//  Open Source Software; you can modify andor share it under the terms of
//  the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Threads;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

  private AprilTagDetector m_detector;
  Thread visionThread;

  public CameraSubsystem() {

    m_detector = new AprilTagDetector();
    m_detector.addFamily("tag36h11");

    visionThread = new Thread(
        () -> {
          UsbCamera camera = CameraServer.startAutomaticCapture();
          camera.setPixelFormat(PixelFormat.kYUYV);
          camera.setFPS(24); // http:roborio-10070-frc.local:1181
          camera.setResolution(320, 240);

          CvSink cvSink = CameraServer.getVideo();

          CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);

          Mat mat = new Mat();
          Mat gray = new Mat(); // grayscale frame

          while (!Thread.interrupted()) {
            if (cvSink.grabFrame(mat) == 0)
              continue; // Skip if no frame

            // april tag detector only works if grayscale imgae
            Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
            var detections = m_detector.detect(gray);

            for (var detection : detections) {
              // Access the ID of the tag (e.g., ID 1-22 for 2026)
              int tagId = detection.getId();

              System.out.println("TagID " + tagId);

              double[] c = detection.getCorners();

              Point p0 = new Point(c[0], c[1]);
              Point p1 = new Point(c[2], c[3]);
              Point p2 = new Point(c[4], c[5]);
              Point p3 = new Point(c[6], c[7]);

              Imgproc.line(mat, p0, p1, new Scalar(0, 0, 255), 2);
              Imgproc.line(mat, p1, p2, new Scalar(0, 0, 255), 2);
              Imgproc.line(mat, p2, p3, new Scalar(0, 0, 255), 2);
              Imgproc.line(mat, p3, p0, new Scalar(0, 0, 255), 2);

              Imgproc.rectangle(gray, new Point(100, 100), new Point(150, 150), new Scalar(130, 130, 130), 5);
              // Give the output stream a new image to display
              // Log or pass tagId to your RobotContainerDrivetrain
            }
            Core.flip(mat, mat, 1);
            outputStream.putFrame(mat);

            mat.release();
            gray.release();
          }
        });
    visionThread.setDaemon(true);
    visionThread.start();

    // enableLiveWindowInTest(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}