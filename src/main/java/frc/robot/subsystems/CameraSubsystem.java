package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.VisionUpdate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Rotation3d;

import java.io.IOException;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public class CameraSubsystem extends SubsystemBase {

    // =========================================================================
    // ⚠️ ROBOT & CAMERA CONSTANTS (YOU MUST TUNE THESE) ⚠️
    // =========================================================================

    // Where is the camera on your robot? (e.g. 0.2 meters forward, 0.5 meters up)
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            new Translation3d(0.3, -0.13, 0.19),
            new Rotation3d(0, Math.toRadians(-20), 0) // Pitched 20 degrees up
    );

    // Camera Calibration Data (Requires calibration using a checkerboard!)
    // These are generic 320x240 example values.
    private static final double FX = 330.0; // Focal length X
    private static final double FY = 330.0; // Focal length Y
    private static final double CX = 160.0; // Center X
    private static final double CY = 120.0; // Center Y

    // 2026 AprilTag size is typically 16.51 cm (0.1651 meters)
    private static final double TAG_SIZE_METERS = 0.1651;

    // =========================================================================

    private AprilTagDetector m_detector;
    private AprilTagPoseEstimator m_poseEstimator;
    private AprilTagFieldLayout m_fieldLayout;
    private Thread visionThread;

    private final AtomicReference<VisionUpdate> latestVisionUpdate = new AtomicReference<>(null);

    public CameraSubsystem() {
        if (RobotBase.isReal()) {
            m_detector = new AprilTagDetector();
            m_detector.addFamily("tag36h11");

            // 1. Initialize the Pose Estimator
            var poseEstConfig = new AprilTagPoseEstimator.Config(TAG_SIZE_METERS, FX, FY, CX, CY);
            m_poseEstimator = new AprilTagPoseEstimator(poseEstConfig);

            // 2. Load the Field Layout map
            try {
                m_fieldLayout = AprilTagFieldLayout
                        .loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
            } catch (IOException e) {
                System.err.println("Failed to load AprilTag Field Layout!");
                m_fieldLayout = null;
            }

            visionThread = new Thread(() -> {
                UsbCamera camera = CameraServer.startAutomaticCapture();
                camera.setPixelFormat(PixelFormat.kYUYV);
                camera.setFPS(20);
                camera.setResolution(320, 240);

                CvSink cvSink = CameraServer.getVideo();
                CvSource outputStream = CameraServer.putVideo("Vision", 320, 240);

                Mat mat = new Mat();
                Mat gray = new Mat();

                while (!Thread.interrupted()) {
                    long frameTimeMicroseconds = cvSink.grabFrame(mat);
                    if (frameTimeMicroseconds == 0)
                        continue;

                    double timestampSeconds = frameTimeMicroseconds / 1_000_000.0;

                    Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
                    var detections = m_detector.detect(gray);

                    // --- NEW POSE MATH CALCULATION ---
                    for (var detection : detections) {
                        // Filter out ghost tags (A margin > 30 usually means it's a solid read)
                        if (detection.getDecisionMargin() < 30)
                            continue;

                        int tagId = detection.getId();

                        // Ensure we have the field layout and the tag exists on the field
                        if (m_fieldLayout != null && m_fieldLayout.getTagPose(tagId).isPresent()) {

                            Transform3d cameraToTag = m_poseEstimator.estimate(detection);

                            // NEW: Get the straight-line distance to the tag
                            double distanceToTag = cameraToTag.getTranslation().getNorm();

                            // Optional: Completely ignore tags that are farther than 4 meters
                            // because they are usually too noisy to be useful!
                            if (distanceToTag > 4.0) {
                                continue; // Skip this detection and check the next one
                            }

                            // 2. Look up exactly where the tag is bolted on the field
                            Pose3d fieldToTag = m_fieldLayout.getTagPose(tagId).get();

                            // 3. Calculate Robot Field Pose (same as before)
                            Pose3d fieldToCamera = fieldToTag.transformBy(cameraToTag.inverse());
                            Pose3d fieldToRobot = fieldToCamera.transformBy(ROBOT_TO_CAMERA.inverse());
                            Pose2d robotPose2d = fieldToRobot.toPose2d();

                            // NEW: Calculate the dynamic standard deviations!
                            Matrix<N3, N1> stdDevs = getVisionStdDevs(distanceToTag);

                            // Add the standard deviations to your record
                            latestVisionUpdate.set(new VisionUpdate(robotPose2d, timestampSeconds, stdDevs));
                            break;
                        }
                    }

                    // --- Drawing Code ---
                    for (var detection : detections) {
                        if (detection.getDecisionMargin() > 30) {
                            double[] c = detection.getCorners();
                            Imgproc.line(mat, new Point(c[0], c[1]), new Point(c[2], c[3]), new Scalar(0, 255, 0), 2);
                            Imgproc.line(mat, new Point(c[2], c[3]), new Point(c[4], c[5]), new Scalar(0, 255, 0), 2);
                            Imgproc.line(mat, new Point(c[4], c[5]), new Point(c[6], c[7]), new Scalar(0, 255, 0), 2);
                            Imgproc.line(mat, new Point(c[6], c[7]), new Point(c[0], c[1]), new Scalar(0, 255, 0), 2);
                        }
                    }

                    // Flip if camera is upside down, otherwise remove this
                    // Core.flip(mat, mat, 1);
                    outputStream.putFrame(mat);
                }
            });
            visionThread.setDaemon(true);
            visionThread.start();
        }
    }

    public Optional<VisionUpdate> getLatestVisionData() {
        VisionUpdate update = latestVisionUpdate.getAndSet(null);
        return Optional.ofNullable(update);
    }

    private Matrix<N3, N1> getVisionStdDevs(double distanceMeters) {
        // Base trust (when you are super close to the tag)
        double baseTranslationStdDev = 0.1; // meters
        double baseRotationStdDev = 0.1; // radians

        // How much the error grows per meter of distance
        // Using distance^2 makes trust drop off rapidly the further you get
        double distanceMultiplier = 0.05;

        double translationStdDev = baseTranslationStdDev + (Math.pow(distanceMeters, 2) * distanceMultiplier);
        double rotationStdDev = baseRotationStdDev + (Math.pow(distanceMeters, 2) * distanceMultiplier);

        // Return a WPILib Matrix (X, Y, Theta)
        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }
}