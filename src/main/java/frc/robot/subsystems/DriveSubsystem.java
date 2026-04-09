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
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.vision.VisionUpdate;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

import java.util.Optional;
import java.util.function.Supplier;

public class DriveSubsystem extends SubsystemBase {
    // SysID
    private final SysIdRoutine m_sysIdRoutine;

    // gyro
    private final Pigeon2 m_gyro;
    private final Pigeon2SimState m_gyroSimState;

    // track robot field location for dashboard
    private final Field2d m_field = new Field2d();

    private boolean m_closedLoopMode = true; // false = Open Loop, true = Velocity PID
    private boolean m_halfSpeedMode = false;
    private double m_speedMultiplier = 1.0;
    private boolean m_directionInverted = false;

    private static final double SIM_DT_SECONDS = 0.02;

    // Heading Correction
    private final PIDController m_headingPID = new PIDController(0.02, 0, 0.001); // Tune kP (0.01 - 0.05)

    // motors
    private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    private final SparkMax leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // encoders
    private final RelativeEncoder m_encoderLeftLeader = leftLeader.getEncoder();
    private final RelativeEncoder m_encoderRightLeader = rightLeader.getEncoder();

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
    private SparkMaxSim m_leftSim;
    private SparkMaxSim m_rightSim;

    private DifferentialDrivetrainSim m_driveTrainSim;

    private SparkRelativeEncoderSim m_leftEncoderSim;
    private SparkRelativeEncoderSim m_rightEncoderSim;

    private final Supplier<Optional<VisionUpdate>> visionSupplier;

    public DriveSubsystem(Supplier<Optional<VisionUpdate>> visionUpdateSupplier) {

        this.visionSupplier = visionUpdateSupplier;

        SmartDashboard.putBoolean("Closed Loop Mode", m_closedLoopMode);

        // Creates a SysIdRoutine
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism((voltage) -> this.setVoltage(voltage, voltage), this::logMotors, this));

        // init gyro
        m_gyro = new Pigeon2(PIGEON2_ID);
        m_gyroSimState = m_gyro.getSimState();

        SmartDashboard.putData("Gyro", m_gyro);

        // Configure Heading PID
        m_headingPID.enableContinuousInput(0, 360);
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
                ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        leftFollower.configure(new SparkMaxConfig().apply(baseDriveConfig).follow(leftLeader),
                ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        rightLeader.configure(new SparkMaxConfig().apply(baseDriveConfig).inverted(true),
                ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        rightFollower.configure(new SparkMaxConfig().apply(baseDriveConfig).follow(rightLeader),
                ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        resetEncoders();

        // init odometry
        m_driveOdometry = new DifferentialDrivePoseEstimator(
                kDriveKinematics,
                m_gyro.getRotation2d(),
                m_encoderLeftLeader.getPosition(),
                m_encoderRightLeader.getPosition(),
                new Pose2d());

        if (RobotBase.isSimulation()) {
            m_leftSim = new SparkMaxSim(leftLeader, DCMotor.getNEO(2));
            m_rightSim = new SparkMaxSim(rightLeader, DCMotor.getNEO(2));

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
                    null);
            // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

            // setup simulated encoders
            m_leftEncoderSim = new SparkRelativeEncoderSim(leftLeader);
            m_rightEncoderSim = new SparkRelativeEncoderSim(rightLeader);
        }
        // Configure AutoBuilder
        AutoBuilder.configure(
                // getPose
                () -> m_driveOdometry.getEstimatedPosition(),
                this::resetPose,
                // getSpeeds
                () -> kDriveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
                        m_encoderLeftLeader.getVelocity(), m_encoderRightLeader.getVelocity())),
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
        resetPose(FieldConstants.START_IN_FRONT_OF_BLUE_HUB);

        SmartDashboard.putData("Field", m_field);

    }

    private void setVoltage(Voltage leftVoltage, Voltage rightVoltage) {
        leftLeader.setVoltage(leftVoltage.in(Volts));
        rightLeader.setVoltage(rightVoltage.in(Volts));
        m_diffDrive.feed();
    }

    private void logMotors(SysIdRoutineLog log) {

        // Calculate actual applied voltage
        double leftVoltage = leftLeader.getAppliedOutput() * RobotController.getBatteryVoltage();
        double rightVoltage = rightLeader.getAppliedOutput() * RobotController.getBatteryVoltage();

        log.motor("drive-left")
                .voltage(Volts.of(leftVoltage))
                .linearPosition(Meters.of(m_encoderLeftLeader.getPosition()))
                .linearVelocity(MetersPerSecond.of(m_encoderLeftLeader.getVelocity()));

        log.motor("drive-right")
                .voltage(Volts.of(rightVoltage))
                .linearPosition(Meters.of(m_encoderRightLeader.getPosition()))
                .linearVelocity(MetersPerSecond.of(m_encoderRightLeader.getVelocity()));
    }

    public void resetPose(Pose2d pose) {
        resetEncoders();
        if (RobotBase.isSimulation() && m_driveTrainSim != null) {
            m_driveTrainSim.setPose(pose);
        }

        m_driveOdometry.resetPosition(
                pose.getRotation(),
                0.0,
                0.0,
                pose);
    }

    public void arcadeDrive(double fwd, double rot) {
        m_diffDrive.arcadeDrive(fwd, rot, false);
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

    public Command setMaxSpeed(double percent) {
        return runOnce(
                () -> {
                    m_speedMultiplier = percent;
                    // System.out.println("Setting Max Speed to " + (percent * 100) + "%");
                    SmartDashboard.putString("Speed Multiplier", (percent * 100) + "%");
                });
    }

    public void toggleDirection() {
        m_directionInverted = !m_directionInverted;
        SmartDashboard.putBoolean("Inverted Direction", m_directionInverted);
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

        fwdSpeed *= m_speedMultiplier;
        rotSpeed *= m_speedMultiplier;

        if (m_directionInverted) {
            fwdSpeed *= -1;
        }

        if (m_closedLoopMode) {
            // Closed Loop: Positive finalRot turns CCW (Left)
            driveVelocity(fwdSpeed, rotSpeed);
            m_diffDrive.feed();
        } else {
            // Open Loop: arcadeDrive expects positive to be CW (Right).
            m_diffDrive.arcadeDrive(fwdSpeed, rotSpeed, false);
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

    public Command driveDistanceCommand(double distanceMeters, double speed) {
        final double commandedSpeed = Math.copySign(Math.abs(speed), distanceMeters);
        final double[] start = new double[2];
        return runOnce(() -> {
            start[0] = m_encoderLeftLeader.getPosition();
            start[1] = m_encoderRightLeader.getPosition();
        })
                .andThen(run(() -> m_diffDrive.arcadeDrive(commandedSpeed, 0, false)))
                .until(() -> Math.max(Math.abs(m_encoderLeftLeader.getPosition() - start[0]),
                        Math.abs(m_encoderRightLeader.getPosition() - start[1])) >= Math.abs(distanceMeters))
                .finallyDo(interrupted -> m_diffDrive.stopMotor());
    }

    private void resetEncoders() {
        m_encoderLeftLeader.setPosition(0);
        m_encoderRightLeader.setPosition(0);
        if (RobotBase.isSimulation() && m_leftEncoderSim != null && m_rightEncoderSim != null) {
            m_leftEncoderSim.setPosition(0);
            m_rightEncoderSim.setPosition(0);
        }
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

    private double normalizeHeading(double degrees) {
        return MathUtil.inputModulus(degrees, 0, 360);
    }

    private void postCoords() {
        Pose2d pose = m_driveOdometry.getEstimatedPosition();

        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        SmartDashboard.putNumber("Robot Rot", normalizeHeading(pose.getRotation().getDegrees()));
    }

    private int m_loopCount = 0;

    @Override
    public void periodic() {
        // so genuinely no clue what's happening, but there's a weird inversion
        // somewhere I can't figure out
        // so for right now we'll just read from the drivetrainsim id
        // TODO: fix
        Rotation2d currentRotation;
        if (RobotBase.isSimulation()) {
            currentRotation = m_driveTrainSim.getHeading();
        } else {
            currentRotation = m_gyro.getRotation2d();
        }

        m_driveOdometry.update(currentRotation,
                m_encoderLeftLeader.getPosition(),
                m_encoderRightLeader.getPosition());
        m_field.setRobotPose(m_driveOdometry.getEstimatedPosition());

        visionSupplier.get().ifPresent(data ->
            m_driveOdometry.addVisionMeasurement(
                data.pose(),
                data.timestamp(),
                data.stdDevs()
            )
        );

        // Only update dashboard every 100ms
        if (m_loopCount++ % 5 == 0) {
            postCoords();
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

        double batteryVoltage = RoboRioSim.getVInVoltage();

        m_driveTrainSim.setInputs(
                m_leftSim.getAppliedOutput() * batteryVoltage,
                m_rightSim.getAppliedOutput() * batteryVoltage);

        // move sim along
        m_driveTrainSim.update(SIM_DT_SECONDS);

        m_leftSim.iterate(
                m_driveTrainSim.getLeftVelocityMetersPerSecond(), batteryVoltage, SIM_DT_SECONDS);
        m_rightSim.iterate(
                m_driveTrainSim.getRightVelocityMetersPerSecond(), batteryVoltage, SIM_DT_SECONDS);

        // add load to battery
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_driveTrainSim.getCurrentDrawAmps()));

        // update sensors
        m_gyroSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
        m_gyroSimState.setRawYaw(-m_driveTrainSim.getHeading().getDegrees());

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

    public void resetGyro() {
        m_gyro.reset();
    }

}