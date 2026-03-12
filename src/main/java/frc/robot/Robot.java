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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
@SuppressWarnings("unused")
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    private Command m_autonomousCommand;
    private long m_simStartTime;
    private boolean m_simMatchRunning = false;

    private RobotContainer m_robotContainer;

    private AprilTagDetector m_detector;
    Thread visionThread;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        
        if (SmartDashboard.getBoolean("Simulate Match Cycle", false)) {
            m_simStartTime = System.currentTimeMillis();
            m_simMatchRunning = true;
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
        MatchStatusPublisher.publish();
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

    @Override
    public void simulationInit() {
        // Mock being connected to a field / FMS
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
        SmartDashboard.setDefaultBoolean("Simulate Match Cycle", false);
        DriverStationSim.notifyNewData();
    }

    @Override
    public void simulationPeriodic() {
        if (m_simMatchRunning) {
            double elapsedSeconds = (System.currentTimeMillis() - m_simStartTime) / 1000.0;
            double matchTimeRemaining = 150.0 - elapsedSeconds;

            if (matchTimeRemaining <= 135.0 && DriverStationSim.getAutonomous()) {
                // Transition to Teleop
                DriverStationSim.setAutonomous(false);
            }

            if (matchTimeRemaining < 0) {
                DriverStationSim.setEnabled(false);
                matchTimeRemaining = 0;
                m_simMatchRunning = false;
            }

            DriverStationSim.setMatchTime(matchTimeRemaining);
            DriverStationSim.notifyNewData(); // Make sure the station pushes the simulated updates
        }
        MatchStatusPublisher.publish();
    }
}