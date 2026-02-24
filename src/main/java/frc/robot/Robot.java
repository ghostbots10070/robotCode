/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private AprilTagDetector m_detector;
    Thread visionThread;

    @Override
    public void robotInit() {
        /*
         * m_detector = new AprilTagDetector();
         * m_detector.addFamily("tag36h11");
         * 
         * visionThread = new Thread(
         * () -> {
         * UsbCamera camera = CameraServer.startAutomaticCapture();
         * //camera.setPixelFormat(PixelFormat.kYUYV);
         * //camera.setFPS(24); // http://roborio-10070-frc.local:1181/
         * camera.setResolution(320, 240);
         * 
         * CvSink cvSink = CameraServer.getVideo();
         * 
         * CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);
         * 
         * Mat mat = new Mat();
         * Mat gray = new Mat(); // grayscale frame
         * 
         * while (!Thread.interrupted()) {
         * if (cvSink.grabFrame(mat) == 0) continue; // Skip if no frame
         * 
         * // april tag detector only works if grayscale imgae
         * Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
         * var detections = m_detector.detect(gray);
         * 
         * for (var detection : detections) {
         * // Access the ID of the tag (e.g., ID 1-22 for 2026)
         * int tagId = detection.getId();
         * 
         * System.out.println("TagID "+ tagId);
         * 
         * double[] c = detection.getCorners();
         * 
         * Point p0 = new Point(c[0], c[1]);
         * Point p1 = new Point(c[2], c[3]);
         * Point p2 = new Point(c[4], c[5]);
         * Point p3 = new Point(c[6], c[7]);
         * 
         * Imgproc.line(mat, p0, p1, new Scalar(0, 0, 255), 2);
         * Imgproc.line(mat, p1, p2, new Scalar(0, 0, 255), 2);
         * Imgproc.line(mat, p2, p3, new Scalar(0, 0, 255), 2);
         * Imgproc.line(mat, p3, p0, new Scalar(0, 0, 255), 2);
         * 
         * //Imgproc.rectangle(grey, new Point(100, 100), new Point(150, 150), new
         * Scalar(130, 130, 130), 5);
         * // Give the output stream a new image to display
         * // Log or pass tagId to your RobotContainer/Drivetrain
         * }
         * Core.flip(mat, mat, 1);
         * outputStream.putFrame(mat);
         * 
         * //source.release();
         * //grey.release();
         * }
         * });
         * visionThread.setDaemon(true);
         * visionThread.start();
         */
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
        // System.out.println("called scheduler");

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
        // if (m_robotContainer.enableAutoProfiling) {
        // System.out.println("WARNING, AUTO PROFILE IS ENABLED!");
        // }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

}