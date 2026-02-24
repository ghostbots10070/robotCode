package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // SysID
    private final SysIdRoutine m_sysIdRoutine;

    // gyro
    private final AHRS m_gyro;

    // track robot field location for dashboard
    private boolean gyroZeroPending = true;
    private final Field2d m_field = new Field2d();

    private boolean m_closedLoopMode = false; // false = Open Loop, true = Velocity PID
    private boolean m_halfSpeedMode = false;
    // Heading Correction
    private final PIDController m_headingPID = new PIDController(0.02, 0, 0); // Tune kP (0.01 - 0.05)
    private double m_targetHeading = 0.0;

    // motors
    private final SparkMax neoMotor;

    private final SparkMax leftLeader = new SparkMax(1, MotorType.kBrushless);;
    private final SparkMax leftFollower = new SparkMax(2, MotorType.kBrushless);;
    private final SparkMax rightLeader = new SparkMax(3, MotorType.kBrushless);;
    private final SparkMax rightFollower = new SparkMax(4, MotorType.kBrushless);;

    // motor configs
    private final SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    private final SparkMaxConfig neoConfig = new SparkMaxConfig();

    // encoders
    private final RelativeEncoder m_encoderleftLeader = leftLeader.getEncoder();
    private final RelativeEncoder m_encoderrightLeader = rightLeader.getEncoder();

    // motor pid controllers
    private final SparkClosedLoopController m_leftLeaderPIDController = leftLeader.getClosedLoopController();
    private final SparkClosedLoopController m_rightLeaderPIDController = rightLeader.getClosedLoopController();

    // main drive function
    private final DifferentialDrive m_diffDrive = new DifferentialDrive(leftLeader, rightLeader);;

    // odometry class for tracking robot on field
    private final DifferentialDrivePoseEstimator m_driveOdometry;

    // motor feedforward
    SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
            DriveConstants.ksDriveVolts,
            DriveConstants.kvDriveVoltSecondsPerMeter,
            DriveConstants.kaDriveVoltSecondsSquaredPerMeter);

    // sim stuff

    private final SimDouble SimGyroAngleHandler;

    private final DCMotor m_leftGearbox;
    private final DCMotor m_rightGearbox;
    private final SparkMaxSim m_leftSim;
    private final SparkMaxSim m_rightSim;

    DifferentialDrivetrainSim m_driveTrainSim;

    SparkRelativeEncoderSim m_leftEncoderSim;
    SparkRelativeEncoderSim m_rightEncoderSim;

    public DriveSubsystem() {
        // Creates a SysIdRoutine
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism((voltage) -> this.setVoltage(voltage, voltage), this::logMotors, this));

        // init gyro
        m_gyro = new AHRS(NavXComType.kMXP_SPI);

        // Configure Heading PID
        m_headingPID.enableContinuousInput(-180, 180);
        m_headingPID.setTolerance(1.0);

        // init motor
        neoMotor = new SparkMax(5, MotorType.kBrushless);

        rightLeaderConfig.inverted(true); // Check your real robot inversion
        rightFollowerConfig.inverted(true); // Followers usually match leader setting automatically via .follow()

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
        // Also fixes FeedForward calculation scaling>
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

        m_leftGearbox = DCMotor.getNEO(2);
        m_rightGearbox = DCMotor.getNEO(2);
        m_leftSim = new SparkMaxSim(leftLeader, m_leftGearbox);
        m_rightSim = new SparkMaxSim(rightLeader, m_rightGearbox);

        // setup simulation for gyro
        int gyroID = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
        SimGyroAngleHandler = new SimDouble(SimDeviceDataJNI.getSimValueHandle(gyroID, "Yaw"));

        m_driveTrainSim = new DifferentialDrivetrainSim(
                // Create a linear system from our identification gains.
                LinearSystemId.identifyDrivetrainSystem(
                        DriveConstants.kvDriveVoltSecondsPerMeter,
                        DriveConstants.kaDriveVoltSecondsSquaredPerMeter,
                        DriveConstants.kvDriveVoltSecondsPerMeterAngular,
                        DriveConstants.kaDriveVoltSecondsSquaredPerMeterAngular),
                DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
                DriveConstants.GEAR_RATIO, // x to 1 gearing reduction
                DriveConstants.kTrackwidthMeters, // Track Width
                DriveConstants.WHEEL_RADIUS, // Wheel Radius
                // The standard deviations for measurement noise:
                // x and y: 0.001 m
                // heading: 0.001 rad
                // l and r velocity: 0.1 m/s
                // l and r position: 0.005 m
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

        // setup simulated encoders
        m_leftEncoderSim = new SparkRelativeEncoderSim(leftLeader);
        m_rightEncoderSim = new SparkRelativeEncoderSim(rightLeader);

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
                this);

        PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", m_field);
    }

    public void setVoltage(Voltage rightVoltage, Voltage leftVoltage) {
        leftLeader.setVoltage(leftVoltage.in(Volts));
        rightLeader.setVoltage(rightVoltage.in(Volts));
        m_diffDrive.feed();
    }

    private void logMotors(SysIdRoutineLog log) {

        // Calculate actual applied voltage
        double leftVoltage = leftLeader.getAppliedOutput() * RobotController.getBatteryVoltage();

        double rightVoltage = rightLeader.getAppliedOutput() * RobotController.getBatteryVoltage();

        // log.motor("left")
        // .voltage(new Voltage(leftVoltage))
        // .linearPosition(getPositionLeft())
        // .linearVelocity(getVelocityLeft());

        // log.motor("right")
        // .voltage(new Voltage(rightVoltage))
        // .linearPosition(getPositionRight())
        // .linearVelocity(getVelocityRight());
    }

    public void resetPose(Pose2d pose) {
        resetEncoders();
        // Reset drivetrain simulation state
        m_driveTrainSim.setPose(pose);
        m_leftEncoderSim.setVelocity(0);
        m_rightEncoderSim.setVelocity(0);
        // Reset odometry AFTER sim pose
        m_driveOdometry.resetPosition(
                pose.getRotation(),
                0.0,
                0.0,
                pose);
    }

    public void arcadeDrive(double fwd, double rot) {
        m_diffDrive.arcadeDrive(fwd, rot);
    }

    /**
     * Toggles between Open Loop (Voltage) and Closed Loop (Velocity PID) control
     */
    public void toggleDriveMode() {
        m_closedLoopMode = !m_closedLoopMode;
        SmartDashboard.putBoolean("Closed Loop Mode", m_closedLoopMode);
    }
    public void toggleHalfSpeed() {
        m_halfSpeedMode = !m_halfSpeedMode;
        SmartDashboard.putBoolean("Slow Mode", m_halfSpeedMode);
    }

    /**
     * MAIN DRIVE METHOD
     * Handles: Half Speed, Closed/Open Loop Toggle, and Gyro Correction
     */
    public void drive(double fwdSpeed, double rotSpeed) {

        // 1. Gyro Stabilization (Heading Lock)
        // If driver isn't steering, use PID to keep straight.
        // If driver IS steering, update the target heading to current.

        if (m_halfSpeedMode) {
            fwdSpeed *= 0.5;
            rotSpeed *= 0.5;
        }

        double finalRot = rotSpeed;

        // Only attempt stabilization if we are moving forward but not turning
        if (Math.abs(rotSpeed) < 0.05 && Math.abs(fwdSpeed) > 0.05) {
            // Calculate correction
            double correction = m_headingPID.calculate(getRotation2d().getDegrees(), m_targetHeading);
            // Clamp to prevent violent shaking
            finalRot = MathUtil.clamp(correction, -0.5, 0.5);
        } else {
            // Update target heading so we lock to this new angle when we stop turning
            m_targetHeading = getRotation2d().getDegrees();
        }

        // 2. Drive Execution
        SmartDashboard.putBoolean("Mode: Closed Loop", m_closedLoopMode);

        if (m_closedLoopMode) {
            // Closed Loop: Velocity Control (Consistent Speed regardless of battery)
            driveVelocity(fwdSpeed, finalRot);
            m_diffDrive.feed(); // Safety watchdog
        } else {
            // Open Loop: Standard Arcade (Raw Voltage)
            m_diffDrive.arcadeDrive(fwdSpeed, finalRot);
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
        m_encoderrightLeader.setPosition(0);
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
        // Since voltageCompensation is 12.0, the SparkMax handles the Volts to Duty
        // Cycle conversion
        m_leftLeaderPIDController.setSetpoint(
                leftSpeed,
                SparkBase.ControlType.kVelocity,
                DriveConstants.kDrivetrainVelocityPIDSlot,
                leftFF);

        m_rightLeaderPIDController.setSetpoint(
                rightSpeed,
                SparkBase.ControlType.kVelocity,
                DriveConstants.kDrivetrainVelocityPIDSlot,
                rightFF);
    }

    private void configureMotorPIDControllers() {
        applyPIDConfig(leftLeaderConfig);
        applyPIDConfig(rightLeaderConfig);
    }

    // Helper method to apply identical settings to any config object
    private void applyPIDConfig(SparkMaxConfig config) {
        // Velocity Slot
        config.closedLoop
                .pid(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel, DriveConstants.kDDriveVel,
                        DriveConstants.kDrivetrainVelocityPIDSlot)
                .iZone(DriveConstants.kIzDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot)
                .outputRange(DriveConstants.kMinOutputDrive, DriveConstants.kMaxOutputDrive,
                        DriveConstants.kDrivetrainVelocityPIDSlot);

        // Position Slot
        config.closedLoop
                .pid(DriveConstants.kPDrivePos, DriveConstants.kIDrivePos, DriveConstants.kDDrivePos,
                        DriveConstants.kDrivetrainPositionPIDSlot)
                .iZone(DriveConstants.kIzDrivePos, DriveConstants.kDrivetrainPositionPIDSlot)
                .outputRange(DriveConstants.kMinOutputDrive, DriveConstants.kMaxOutputDrive,
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

        // double x = m_gyro.getDisplacementX();
        // double y = m_gyro.getDisplacementY();

        // SmartDashboard.putNumber("navX X", x);
        // SmartDashboard.putNumber("navX Y", y);

        postCords();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        // link motors to simulation
        m_driveTrainSim.setInputs(
                m_leftSim.getAppliedOutput() * RobotController.getInputVoltage(),
                m_rightSim.getAppliedOutput() * RobotController.getInputVoltage());
        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        m_driveTrainSim.update(0.02);
        // update spark maxes
        m_leftSim.iterate(
                m_driveTrainSim.getLeftVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);
        m_rightSim.iterate(
                m_driveTrainSim.getRightVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);
        // add load to battery
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_driveTrainSim.getCurrentDrawAmps()));
        // update sensors
        SimGyroAngleHandler.set(-m_driveTrainSim.getHeading().getDegrees());
        m_leftEncoderSim.setPosition(m_driveTrainSim.getLeftPositionMeters());
        m_leftEncoderSim.setVelocity(m_driveTrainSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setPosition(m_driveTrainSim.getRightPositionMeters());
        m_rightEncoderSim.setVelocity(m_driveTrainSim.getRightVelocityMetersPerSecond());
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

}