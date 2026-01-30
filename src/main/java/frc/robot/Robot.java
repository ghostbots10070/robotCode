/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
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
  private Thread visionThread;

  @Override
public void robotInit() {
    //Camera Stuff
    CameraServer.startAutomaticCapture();

    m_detector = new AprilTagDetector();
    m_detector.addFamily("tag36h11");

    visionThread = new Thread(() -> {
          CvSink cvSink = CameraServer.getVideo();
          Mat source = new Mat();
          Mat grey = new Mat();

          while (!Thread.interrupted()) {
            if (cvSink.grabFrame(source) == 0) continue; // Skip if no frame

            // Convert to grayscale for faster processing
            Imgproc.cvtColor(source, grey, Imgproc.COLOR_BGR2GRAY);

            // Run detection
            var detections = m_detector.detect(grey);
          
            for (var detection : detections) {
              // Access the ID of the tag (e.g., ID 1-22 for 2026)
              int tagId = detection.getId();
              System.out.println("TagID "+ tagId);
              // Log or pass tagId to your RobotContainer/Drivetrain
            }
              source.release();
              grey.release();
          }
        });
        visionThread.start();

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
    //   System.out.println("WARNING, AUTO PROFILE IS ENABLED!");
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