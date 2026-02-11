package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // gyro
    private final AHRS m_gyro;

    // track robot field location for dashboard
    private boolean gyroZeroPending = true;
    private final Field2d m_field = new Field2d();

    // Logic Flags
    private boolean halfSpeed = false;
    private boolean lastHalfInput = false; // For rising edge detection
    
    private boolean m_closedLoopMode = false; // false = Open Loop, true = Velocity PID
    private boolean m_lastToggleInput = false; // For rising edge detection

    // Heading Correction
    private final PIDController m_headingPID = new PIDController(0.02, 0, 0); // Tune kP (0.01 - 0.05)
    private double m_targetHeading = 0.0;
    private boolean m_wasTurning = false;

    // motors
    private final SparkMax neoMotor;

    private final SparkMax leftLeader;
    private final SparkMax leftFollower;
    private final SparkMax rightLeader;
    private final SparkMax rightFollower;

    // motor configs
    private final SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    private final SparkMaxConfig neoConfig = new SparkMaxConfig();

    // encoders
    private final RelativeEncoder m_encoderleftLeader;
    private final RelativeEncoder m_encoderleftFollower;
    private final RelativeEncoder m_encoderrightLeader;
    private final RelativeEncoder m_encoderrightFollower;

    // motor pid controllers
    private final SparkClosedLoopController m_leftLeaderPIDController;
    private final SparkClosedLoopController m_rightLeaderPIDController;

    // main drive function
    private final DifferentialDrive myDrive;

    // odometry class for tracking robot on field
    private final DifferentialDrivePoseEstimator m_driveOdometry;

    // motor feedforward
    SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
            DriveConstants.ksDriveVolts,
            DriveConstants.kvDriveVoltSecondsPerMeter,
            DriveConstants.kaDriveVoltSecondsSquaredPerMeter);

    public DriveSubsystem() {

        // init gyro
        m_gyro = new AHRS(NavXComType.kMXP_SPI);
        
        // Configure Heading PID
        m_headingPID.enableContinuousInput(-180, 180);
        m_headingPID.setTolerance(1.0);

        // init motor
        neoMotor = new SparkMax(5, MotorType.kBrushless);

        leftLeader = new SparkMax(1, MotorType.kBrushless);
        leftFollower = new SparkMax(2, MotorType.kBrushless);
        rightLeader = new SparkMax(3, MotorType.kBrushless);
        rightFollower = new SparkMax(4, MotorType.kBrushless);

        rightLeaderConfig.inverted(true); // Check your real robot inversion
        rightFollowerConfig.inverted(true); // Followers usually match leader setting automatically via .follow()

        // init drive function
        // Note: DifferentialDrive automatically inverts the right side, so be careful double inverting
        myDrive = new DifferentialDrive(leftLeader, rightLeader);

        // init encoders
        m_encoderleftLeader = leftLeader.getEncoder();
        m_encoderleftFollower = leftFollower.getEncoder();
        m_encoderrightLeader = rightLeader.getEncoder();
        m_encoderrightFollower = rightFollower.getEncoder();

        // init PID Controllers
        m_leftLeaderPIDController = leftLeader.getClosedLoopController();
        m_rightLeaderPIDController = rightLeader.getClosedLoopController();

        // configure encoders
        // RPM TO m/s
        leftLeaderConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
        rightLeaderConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
        leftFollowerConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
        rightFollowerConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
        // rotations to meters
        leftLeaderConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
        rightLeaderConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
        leftFollowerConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
        rightFollowerConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
        
        resetEncoders();

        // setup PID controllers
        configureMotorPIDControllers();

        // setup main and secondary motors
        leftFollowerConfig.follow(leftLeader); 
        rightFollowerConfig.follow(rightLeader); 

        // CRITICAL: Voltage Compensation
        // This ensures 12.0 V in code = 100% output, regardless of battery level
        // Also fixes FeedForward calculation scaling
        leftLeaderConfig.voltageCompensation(12.0);
        rightLeaderConfig.voltageCompensation(12.0);
        leftFollowerConfig.voltageCompensation(12.0);
        rightFollowerConfig.voltageCompensation(12.0);

        neoConfig.smartCurrentLimit(60);
        neoConfig.voltageCompensation(10);

        // burn config to motors
        leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        neoMotor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // init odometry
        m_driveOdometry = new DifferentialDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                getRotation2d(),
                getPositionLeft(),
                getPositionRight(),
                new Pose2d());

        // Configure AutoBuilder
        AutoBuilder.configure(
                this::getPose, 
                this::resetPose, 
                this::getSpeeds, 
                this::setSpeeds, 
                new PPLTVController(0.02), 
                DriveConstants.autoConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", m_field);
    }

    public Pose2d getPose() {
        return m_driveOdometry.getEstimatedPosition();
    }

    public Rotation2d getRotation2d() {
        return m_gyro.getRotation2d();
    }

    public double getPitch() {
        return m_gyro.getPitch(); 
    }

    public double getAccumYaw() {
        return m_gyro.getAngle(); 
    }

    public double getYaw() {
        return m_gyro.getYaw();
    }

    public void resetGyro() {
        m_gyro.reset();
    }

    public double getVelocityLeft() {
        return m_encoderleftLeader.getVelocity();
    }

    public double getVelocityRight() {
        return m_encoderrightLeader.getVelocity();
    }

    public double getPositionLeft() {
        return m_encoderleftLeader.getPosition();
    }

    public double getPositionRight() {
        return m_encoderrightLeader.getPosition();
    }

    public void resetPose(Pose2d pose) {
        resetEncoders();
        m_driveOdometry.resetPosition(getRotation2d(), getPositionLeft(), getPositionRight(), pose);
    }

    public void arcadeDrive(double fwd, double rot) {
        myDrive.arcadeDrive(fwd, rot);
    }

    /**
     * MAIN DRIVE METHOD
     * Handles: Half Speed, Closed/Open Loop Toggle, and Gyro Correction
     */
    public void drive(boolean half, 
                      boolean toggleMode, // Logic for toggling drive modes
                      double fwdSpeed, double rotSpeed) {
        
        // 1. Toggle Half Speed (Rising Edge)
        if (half && !lastHalfInput) {
            halfSpeed = !halfSpeed;
        }
        lastHalfInput = half;

        // 2. Toggle Drive Mode (Rising Edge)
        if (toggleMode && !m_lastToggleInput) {
            m_closedLoopMode = !m_closedLoopMode;
        }
        m_lastToggleInput = toggleMode;

        // Dashboard Info
        SmartDashboard.putBoolean("Mode: Closed Loop", m_closedLoopMode);
        SmartDashboard.putBoolean("Mode: Half Speed", halfSpeed);

        // 3. Apply Divider
        int divider = halfSpeed ? 2 : 1;
        double finalFwd = fwdSpeed / divider;
        double finalRot = rotSpeed / divider;

        // 4. Gyro Stabilization / Heading Lock
        // If driver stops rotating, lock to current heading using PID
        double rotationInput = MathUtil.applyDeadband(finalRot, 0.05);
        double correctedRotation;

        if (Math.abs(rotationInput) > 0.0) {
            // Driver is steering manually
            correctedRotation = rotationInput;
            m_targetHeading = getRotation2d().getDegrees();
            m_wasTurning = true;
        } else {
            // Driver wants to go straight - use PID
            if (m_wasTurning) {
                m_targetHeading = getRotation2d().getDegrees();
                m_wasTurning = false;
            }
            // Calculate correction
            correctedRotation = m_headingPID.calculate(getRotation2d().getDegrees(), m_targetHeading);
            // Clamp output to prevent violent shakes
            correctedRotation = MathUtil.clamp(correctedRotation, -0.5, 0.5);
        }

        // 5. Send to Motor Control
        if (m_closedLoopMode) {
            // Velocity Control (Meters Per Second)
            driveVelocity(finalFwd, correctedRotation);
        } else {
            // Voltage/Percent Control
            arcadeDrive(finalFwd, correctedRotation);
        }
    }

    /**
     * Helper to convert Joystick inputs to Closed-Loop Velocity commands
     */
    public void driveVelocity(double xSpeed, double zRotation) {
        // Convert inputs (-1 to 1) to m/s and rad/s
        double targetLinear = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double targetAngular = zRotation * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

        // Convert to chassis speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(targetLinear, 0, targetAngular);
        
        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);
        
        // Desaturate (ensure we don't ask for more speed than possible)
        wheelSpeeds.desaturate(DriveConstants.kMaxSpeedMetersPerSecond);

        // Send to motors
        setWheelVelocities(wheelSpeeds);
    }

    public void resetEncoders() {
        m_encoderleftLeader.setPosition(0);
        m_encoderleftFollower.setPosition(0);
        m_encoderrightLeader.setPosition(0);
        m_encoderrightFollower.setPosition(0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        setWheelVelocities(DriveConstants.kDriveKinematics.toWheelSpeeds(speeds));
    }

    public void setWheelVelocities(DifferentialDriveWheelSpeeds speeds) {
        double leftSpeed = speeds.leftMetersPerSecond;
        double rightSpeed = speeds.rightMetersPerSecond;

        // Calculate Feedforward (Returns Volts)
        double leftFF = m_driveFeedForward.calculate(leftSpeed);
        double rightFF = m_driveFeedForward.calculate(rightSpeed);

        // PID + Feedforward
        // Since voltageCompensation is 12.0, the SparkMax handles the Volts to Duty Cycle conversion
        m_leftLeaderPIDController.setReference(
                leftSpeed,
                SparkBase.ControlType.kVelocity,
                DriveConstants.kDrivetrainVelocityPIDSlot,
                leftFF);
        
        m_rightLeaderPIDController.setReference(
                rightSpeed,
                SparkBase.ControlType.kVelocity,
                DriveConstants.kDrivetrainVelocityPIDSlot,
                rightFF);
    }

    private void configureMotorPIDControllers() {
        // setup velocity PID controllers (used by auto and Closed Loop Teleop)
        leftLeaderConfig.closedLoop.pid(
                DriveConstants.kPDriveVel,
                DriveConstants.kIDriveVel,
                DriveConstants.kDDriveVel,
                DriveConstants.kDrivetrainVelocityPIDSlot);
        rightLeaderConfig.closedLoop.pid(
                DriveConstants.kPDriveVel,
                DriveConstants.kIDriveVel,
                DriveConstants.kDDriveVel,
                DriveConstants.kDrivetrainVelocityPIDSlot);
        
        leftLeaderConfig.closedLoop.iZone(
                DriveConstants.kIzDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
        rightLeaderConfig.closedLoop.iZone(
                DriveConstants.kIzDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
        
        leftLeaderConfig.closedLoop.outputRange(
                DriveConstants.kMinOutputDrive,
                DriveConstants.kMaxOutputDrive,
                DriveConstants.kDrivetrainVelocityPIDSlot);
        rightLeaderConfig.closedLoop.outputRange(
                DriveConstants.kMinOutputDrive,
                DriveConstants.kMaxOutputDrive,
                DriveConstants.kDrivetrainVelocityPIDSlot);

        // setup position PID controllers
        leftLeaderConfig.closedLoop.pid(
                DriveConstants.kPDrivePos,
                DriveConstants.kIDrivePos,
                DriveConstants.kDDrivePos,
                DriveConstants.kDrivetrainPositionPIDSlot);
        rightLeaderConfig.closedLoop.pid(
                DriveConstants.kPDrivePos,
                DriveConstants.kIDrivePos,
                DriveConstants.kDDrivePos,
                DriveConstants.kDrivetrainPositionPIDSlot);

        leftLeaderConfig.closedLoop.iZone(
                DriveConstants.kIzDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
        rightLeaderConfig.closedLoop.iZone(
                DriveConstants.kIzDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
        
        leftLeaderConfig.closedLoop.outputRange(
                DriveConstants.kMinOutputDrive,
                DriveConstants.kMaxOutputDrive,
                DriveConstants.kDrivetrainPositionPIDSlot);
        rightLeaderConfig.closedLoop.outputRange(
                DriveConstants.kMinOutputDrive,
                DriveConstants.kMaxOutputDrive,
                DriveConstants.kDrivetrainPositionPIDSlot);
    }

    public void postCords() {
        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("Robot Y", getPose().getY());
        SmartDashboard.putNumber("Robot Rot", getPose().getRotation().getDegrees());
    }

    @Override
    public void periodic() {
        // Fix for infinite reset loop
        if (gyroZeroPending && !m_gyro.isCalibrating()) {
            m_gyro.reset();
            gyroZeroPending = false; // Flag false immediately to prevent re-entry
        }

        m_driveOdometry.update(getRotation2d(), getPositionLeft(), getPositionRight());
        m_field.setRobotPose(getPose());
        
        double x = m_gyro.getDisplacementX();
        double y = m_gyro.getDisplacementY();

        SmartDashboard.putNumber("navX X", x);
        SmartDashboard.putNumber("navX Y", y);

        postCords();
    }
}