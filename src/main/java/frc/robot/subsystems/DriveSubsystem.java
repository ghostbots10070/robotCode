package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
import edu.wpi.first.wpilibj.DataLogManager;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

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
    private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    private final SparkMax leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // encoders
    private final RelativeEncoder m_encoderleftLeader = leftLeader.getEncoder();
    private final RelativeEncoder m_encoderrightLeader = rightLeader.getEncoder();

    // motor pid controllers
    private final SparkClosedLoopController m_leftLeaderPIDController = leftLeader.getClosedLoopController();
    private final SparkClosedLoopController m_rightLeaderPIDController = rightLeader.getClosedLoopController();

    // main drive function
    private final DifferentialDrive m_diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    // odometry class for tracking robot on field
    private final DifferentialDrivePoseEstimator m_driveOdometry;

    // motor feedforward
    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
            ksDriveVolts,
            kvDriveVoltSecondsPerMeter,
            kaDriveVoltSecondsSquaredPerMeter);

    // sim stuff

    private final SimDouble SimGyroAngleHandler;

    private final DCMotor m_leftGearbox;
    private final DCMotor m_rightGearbox;
    private final SparkMaxSim m_leftSim;
    private final SparkMaxSim m_rightSim;

    private final DifferentialDrivetrainSim m_driveTrainSim;

    private final SparkRelativeEncoderSim m_leftEncoderSim;
    private final SparkRelativeEncoderSim m_rightEncoderSim;

    private final Double kSimDt = 0.02; // 20 ms loop time

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

        SparkMaxConfig baseDriveConfig = new SparkMaxConfig();
        baseDriveConfig.voltageCompensation(12.0);

        baseDriveConfig.encoder
                .velocityConversionFactor(VELOCITY_CONVERSION_RATIO)
                .positionConversionFactor(POSITION_CONVERSION_RATIO);

        // You can chain PID slots together to save even more space
        baseDriveConfig.closedLoop
                .pid(kPDriveVel, kIDriveVel, kDDriveVel, kDrivetrainVelocityPIDSlot)
                .iZone(kIzDriveVel, kDrivetrainVelocityPIDSlot)
                .outputRange(kMinOutputDrive, kMaxOutputDrive, kDrivetrainVelocityPIDSlot)
                .pid(kPDrivePos, kIDrivePos, kDDrivePos, kDrivetrainPositionPIDSlot)
                .iZone(kIzDrivePos, kDrivetrainPositionPIDSlot)
                .outputRange(kMinOutputDrive, kMaxOutputDrive, kDrivetrainPositionPIDSlot);

        // 2. Build specific configs and push them to hardware INLINE
        leftLeader.configure(new SparkMaxConfig().apply(baseDriveConfig),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftFollower.configure(new SparkMaxConfig().apply(baseDriveConfig).follow(leftLeader),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightLeader.configure(new SparkMaxConfig().apply(baseDriveConfig).inverted(true),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightFollower.configure(new SparkMaxConfig().apply(baseDriveConfig).follow(rightLeader),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoders();

        // init odometry
        m_driveOdometry = new DifferentialDrivePoseEstimator(
                kDriveKinematics,
                m_gyro.getRotation2d(),
                m_encoderleftLeader.getPosition(),
                m_encoderrightLeader.getPosition(),
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
                        kvDriveVoltSecondsPerMeter,
                        kaDriveVoltSecondsSquaredPerMeter,
                        kvDriveVoltSecondsPerMeterAngular,
                        kaDriveVoltSecondsSquaredPerMeterAngular),
                DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
                GEAR_RATIO, // x to 1 gearing reduction
                kTrackwidthMeters, // Track Width
                WHEEL_RADIUS, // Wheel Radius
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
                // getPose
                () -> m_driveOdometry.getEstimatedPosition(),
                this::resetPose,
                // getSpeeds
                () -> kDriveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
                        m_encoderleftLeader.getVelocity(), m_encoderrightLeader.getVelocity())),
                // setSpeeds
                (ChassisSpeeds speeds) -> setWheelVelocities(kDriveKinematics.toWheelSpeeds(speeds)),
                new PPLTVController(0.02),
                autoConfig,
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

    private void setVoltage(Voltage rightVoltage, Voltage leftVoltage) {
        leftLeader.setVoltage(leftVoltage.in(Volts));
        rightLeader.setVoltage(rightVoltage.in(Volts));
        m_diffDrive.feed();
    }

    private void logMotors(SysIdRoutineLog log) {

        // Calculate actual applied voltage
        double leftVoltage = leftLeader.getAppliedOutput() * RobotController.getBatteryVoltage();
        double rightVoltage = rightLeader.getAppliedOutput() * RobotController.getBatteryVoltage();

        log.motor("left")
                .voltage(Volts.of(leftVoltage))
                .linearPosition(Meters.of(m_encoderleftLeader.getPosition()))
                .linearVelocity(MetersPerSecond.of(m_encoderleftLeader.getVelocity()));

        log.motor("right")
                .voltage(Volts.of(rightVoltage))
                .linearPosition(Meters.of(m_encoderrightLeader.getPosition()))
                .linearVelocity(MetersPerSecond.of(m_encoderrightLeader.getVelocity()));
    }

    public void resetPose(Pose2d pose) {
        resetEncoders();
        // Reset drivetrain simulation state
        // TODO: figure out if we actually neec to do this or it just messes up the
        // whole thing
        m_driveTrainSim.setPose(pose);
        m_leftEncoderSim.setVelocity(0);
        m_rightEncoderSim.setVelocity(0);
        // Reset odometry AFTER sim pose
        m_driveOdometry.resetPosition(
                m_gyro.getRotation2d(),
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
        DataLogManager.log("Toggled Drive Mode: " + (m_closedLoopMode ? "Closed Loop" : "Open Loop"));
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
        if (m_halfSpeedMode) {
            fwdSpeed *= 0.5;
            rotSpeed *= 0.5;
        }

        double finalRot = rotSpeed;

        // Only attempt stabilization if moving forward but not commanding a turn
        if (Math.abs(rotSpeed) < 0.05 && Math.abs(fwdSpeed) > 0.05) {
            if (Math.abs(m_gyro.getRate()) < 2.0) {
                // Calculate correction (Positive output means we need to turn Left/CCW)
                double correction = m_headingPID.calculate(m_gyro.getRotation2d().getDegrees(), m_targetHeading);
                finalRot = MathUtil.clamp(correction, -0.3, 0.3);
            } else {
                m_targetHeading = m_gyro.getRotation2d().getDegrees();
            }
        } else {
            m_targetHeading = m_gyro.getRotation2d().getDegrees();
        }

        SmartDashboard.putBoolean("Mode: Closed Loop", m_closedLoopMode);

        if (m_closedLoopMode) {
            // Closed Loop: Positive finalRot turns CCW (Left)
            driveVelocity(fwdSpeed, finalRot);
            m_diffDrive.feed();
        } else {
            // Open Loop: arcadeDrive expects positive to be CW (Right).
            // CRITICAL FIX: We MUST pass -finalRot so that positive rotSpeed turns CCW
            // (Left)
            // This unifies Open Loop, Closed Loop, and the Gyro PID!
            m_diffDrive.arcadeDrive(fwdSpeed, finalRot, false);
        }
    }

    /**
     * Helper to convert Joystick inputs to Closed-Loop Velocity commands
     */
    private void driveVelocity(double xSpeed, double zRotation) {
        // Convert inputs (-1 to 1) to m/s and rad/s
        double targetLinear = xSpeed * kMaxSpeedMetersPerSecond;
        double targetAngular = zRotation * kMaxAngularSpeedRadiansPerSecond;

        // Convert to chassis speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(targetLinear, 0, targetAngular);

        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kDriveKinematics.toWheelSpeeds(chassisSpeeds);

        // Desaturate (ensure we don't ask for more speed than possible)
        wheelSpeeds.desaturate(kMaxSpeedMetersPerSecond);

        // Send to motors
        setWheelVelocities(wheelSpeeds);
    }

    private void resetEncoders() {
        m_encoderleftLeader.setPosition(0);
        m_encoderrightLeader.setPosition(0);
    }

    private void setWheelVelocities(DifferentialDriveWheelSpeeds speeds) {
        // Calculate Feedforward (Returns Volts)
        double leftFF = m_driveFeedForward.calculate(speeds.leftMetersPerSecond);
        double rightFF = m_driveFeedForward.calculate(speeds.rightMetersPerSecond);

        // PID + Feedforward
        // Since voltageCompensation is 12.0, the SparkMax handles the Volts to Duty
        // Cycle conversion
        m_leftLeaderPIDController.setSetpoint(
                speeds.leftMetersPerSecond,
                SparkBase.ControlType.kVelocity,
                kDrivetrainVelocityPIDSlot,
                leftFF);

        m_rightLeaderPIDController.setSetpoint(
                speeds.rightMetersPerSecond,
                SparkBase.ControlType.kVelocity,
                kDrivetrainVelocityPIDSlot,
                rightFF);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    private void postCords() {
        Pose2d pose = m_driveOdometry.getEstimatedPosition();

        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        SmartDashboard.putNumber("Robot Rot", pose.getRotation().getDegrees());
    }

    // public void updateVisionPose(Pose2d visionRobotPose, double timestamp, String
    // cameraName, boolean cameraEnabled) {
    // if (cameraEnabled) {
    // m_driveOdometry.addVisionMeasurement(visionRobotPose, timestamp);
    // }
    // }

    @Override
    public void periodic() {
        // Fix for infinite reset loop
        if (gyroZeroPending && !m_gyro.isCalibrating()) {
            m_gyro.reset();
            gyroZeroPending = false; // Flag false immediately to prevent re-entry
        }

        m_driveOdometry.update(m_gyro.getRotation2d(), m_encoderleftLeader.getPosition(),
                m_encoderrightLeader.getPosition());
        m_field.setRobotPose(m_driveOdometry.getEstimatedPosition());

        postCords();
    }

    @Override
    public void simulationPeriodic() {

        // 1. Get motor voltages being applied
        double leftVoltage = m_leftSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
        double rightVoltage = m_rightSim.getAppliedOutput() * RoboRioSim.getVInVoltage();

        // 2. Feed voltages into drivetrain physics model
        m_driveTrainSim.setInputs(leftVoltage, rightVoltage);

        // 3. Advance physics simulation
        m_driveTrainSim.update(kSimDt);

        // 4. Simulate battery voltage sag
        double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(
                m_driveTrainSim.getCurrentDrawAmps());
        RoboRioSim.setVInVoltage(loadedVoltage);

        // 5. Update SparkMax simulations
        m_leftSim.iterate(
                m_driveTrainSim.getLeftVelocityMetersPerSecond(),
                loadedVoltage,
                kSimDt);

        m_rightSim.iterate(
                m_driveTrainSim.getRightVelocityMetersPerSecond(),
                loadedVoltage,
                kSimDt);

        // 6. Update simulated encoders
        m_leftEncoderSim.setPosition(m_driveTrainSim.getLeftPositionMeters());
        m_leftEncoderSim.setVelocity(m_driveTrainSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setPosition(m_driveTrainSim.getRightPositionMeters());
        m_rightEncoderSim.setVelocity(m_driveTrainSim.getRightVelocityMetersPerSecond());

        // 7. Update gyro (match real-world sign!)
        SimGyroAngleHandler.set(-m_driveTrainSim.getHeading().getDegrees());
    }

    public Pose2d getPose() {
        return m_driveOdometry.getEstimatedPosition();
    }

    public Rotation2d getRotation2d() {
        return m_gyro.getRotation2d();
    }

    public void resetGyro() {
        m_gyro.reset();
    }
}