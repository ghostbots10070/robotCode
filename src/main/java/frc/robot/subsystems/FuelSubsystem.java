// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.FuelConstants.*;

import java.util.Set;

public class FuelSubsystem extends SubsystemBase {
    private final SparkMax LeftIntakeLauncher;
    private final SparkMax RightIntakeLauncher;
    private final SparkMax Indexer;

    private RelativeEncoder leftIntakeLauncherEncoder;
    private RelativeEncoder rightIntakeLauncherEncoder;

    private SparkClosedLoopController leftIntakeLauncherPID;
    private SparkClosedLoopController rightIntakeLauncherPID;

    private final SparkMaxSim leftLauncherSim;
    private final SparkMaxSim rightLauncherSim;
    private final SparkMaxSim indexerSim;

    private final double kSimDt = 0.02;

    /** Creates a new CANBallSubsystem. */
    public FuelSubsystem() {
        LeftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        RightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);

        leftIntakeLauncherPID = LeftIntakeLauncher.getClosedLoopController();
        rightIntakeLauncherPID = RightIntakeLauncher.getClosedLoopController();

        leftIntakeLauncherEncoder = LeftIntakeLauncher.getEncoder();
        rightIntakeLauncherEncoder = RightIntakeLauncher.getEncoder();

        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
        feederConfig.inverted(false);
        Indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig launcherConfig = new SparkMaxConfig();

        ClosedLoopConfig pidConfig = new ClosedLoopConfig();
        pidConfig.p(0.002);
        pidConfig.i(0.00001);
        pidConfig.d(0.002);
        pidConfig.iZone(400); 


        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig.kV(0.00018);

        pidConfig.apply(ffConfig);

        // In constructor
        SmartDashboard.putNumber("Tuning/kP", 0.002);
        SmartDashboard.putNumber("Tuning/kI", 0.00001);
        SmartDashboard.putNumber("Tuning/kD", 0.002);

        SmartDashboard.putNumber("Tuning/kV", 0.00018);

        launcherConfig.apply(pidConfig);
        launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        launcherConfig.voltageCompensation(12);
        launcherConfig.idleMode(IdleMode.kCoast);

        launcherConfig.encoder.uvwAverageDepth(2)
                .uvwMeasurementPeriod(10);

        launcherConfig.inverted(true);
        RightIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        launcherConfig.inverted(false);
        LeftIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        DCMotor launcherMotor = DCMotor.getNEO(1);
        DCMotor indexerMotor = DCMotor.getNeo550(1);
        leftLauncherSim = new SparkMaxSim(LeftIntakeLauncher, launcherMotor);
        rightLauncherSim = new SparkMaxSim(RightIntakeLauncher, launcherMotor);
        indexerSim = new SparkMaxSim(Indexer, indexerMotor);

        SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
        SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
        SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
        SmartDashboard.putNumber("Launching launcher RPM", LAUNCHING_LAUNCHER_RPM); // <-- replaces percent
        SmartDashboard.putNumber("Launching RPM tolerance", LAUNCHING_RPM_TOLERANCE);
        SmartDashboard.putNumber("Launching spin-up feeder value", INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT);
    }

    public void setIntakeLauncherRoller(double power) {
        LeftIntakeLauncher.set(power);
        RightIntakeLauncher.set(power);
    }

    /**
     * Commands both launcher motors to a target RPM using the onboard velocity PID.
     */
    public void setShooterRPM(double rpm) {
        leftIntakeLauncherPID.setSetpoint(rpm, SparkMax.ControlType.kVelocity);
        rightIntakeLauncherPID.setSetpoint(rpm, SparkMax.ControlType.kVelocity);
    }

    public void setFeederRoller(double power) {
        Indexer.set(power);
    }

    public void stop() {
        Indexer.set(0);
        LeftIntakeLauncher.set(0);
        RightIntakeLauncher.set(0);
    }

    /**
     * Returns true when both launcher motors are within [tolerance] RPM of
     * [targetRPM].
     */
    private boolean launcherAtRPM(double targetRPM, double tolerance) {
        double leftRPM = leftIntakeLauncherEncoder.getVelocity();
        double rightRPM = rightIntakeLauncherEncoder.getVelocity();
        double leftError = Math.abs(leftRPM - targetRPM);
        double rightError = Math.abs(rightRPM - targetRPM);
        boolean atTarget = leftError <= tolerance && rightError <= tolerance;

        SmartDashboard.putNumber("Debug/Fuel/LeftRPM", leftRPM);
        SmartDashboard.putNumber("Debug/Fuel/RightRPM", rightRPM);
        SmartDashboard.putNumber("Debug/Fuel/RPMTarget", targetRPM);
        SmartDashboard.putNumber("Debug/Fuel/LeftRPMError", leftError);
        SmartDashboard.putNumber("Debug/Fuel/RightRPMError", rightError);
        SmartDashboard.putBoolean("Debug/Fuel/AtRPM", atTarget);

        return atTarget;
    }

    public Command shootAtSpeedCommand() {
        return Commands.defer(() -> {
            // These are evaluated exactly ONCE at the moment the button is pressed.
            double targetRPM = SmartDashboard.getNumber("Launching launcher RPM", LAUNCHING_LAUNCHER_RPM);
            double tolerance = SmartDashboard.getNumber("Launching RPM tolerance", LAUNCHING_RPM_TOLERANCE);
            double feederPercent = SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);

            return Commands.sequence(
                    // 1. Set the shooter RPM
                    Commands.runOnce(() -> setShooterRPM(targetRPM), this),

                    // 2. Wait until at speed
                    Commands.waitUntil(() -> launcherAtRPM(targetRPM, tolerance)),

                    // 3. Feed when at speed, wait when recovering
                    Commands.run(() -> {
                        if (launcherAtRPM(targetRPM, tolerance)) {
                            setFeederRoller(feederPercent);
                        } else {
                            setFeederRoller(0);
                        }
                    }, this));
        }, Set.of(this)) // Pass subsystem requirements to defer
                .finallyDo(interrupted -> stop())
                .withName("ShootAtRPM");
    }
// Add these variables to the top of your class to track changes
private double lastKp = 0.0002;
private double lastKi = 0.0;
private double lastKd = 0.0;
private double lastKv = 0.00018;

@Override
public void periodic() {
    // Read the current values
    double p = SmartDashboard.getNumber("Tuning/kP", 0.0002);
    double i = SmartDashboard.getNumber("Tuning/kI", 0.0);
    double d = SmartDashboard.getNumber("Tuning/kD", 0.0);
    double v = SmartDashboard.getNumber("Tuning/kV", 0.00018);

    // ONLY reconfigure if a number was actually changed on the dashboard
    if (p != lastKp || i != lastKi || d != lastKd || v != lastKv) {
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();
        pidConfig.p(p).i(i).d(d);
        
        // Add an I-Zone (explained below)
        pidConfig.iZone(200); 

        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig.kV(v);
        pidConfig.apply(ffConfig);

        SparkMaxConfig config = new SparkMaxConfig();
        config.apply(pidConfig);
        
        LeftIntakeLauncher.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        RightIntakeLauncher.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Update tracking variables
        lastKp = p; lastKi = i; lastKd = d; lastKv = v;
    }

SmartDashboard.putNumber("Fuel/Indexer Current", Indexer.getOutputCurrent());
        SmartDashboard.putNumber("Fuel/Left RPM", leftIntakeLauncherEncoder.getVelocity());
        SmartDashboard.putNumber("Fuel/Right RPM", rightIntakeLauncherEncoder.getVelocity());

        SmartDashboard.putBoolean("Fuel/Left Connected", LeftIntakeLauncher.getBusVoltage() > 1);
        SmartDashboard.putBoolean("Fuel/Right Connected", RightIntakeLauncher.getBusVoltage() > 1);
        SmartDashboard.putBoolean("Fuel/Indexer Connected", Indexer.getBusVoltage() > 1);}

@Override
    public void simulationPeriodic() {
        double batteryVoltage = RoboRioSim.getVInVoltage();

        // Theoretical free-speed RPMs for REV motors
        double NEO_MAX_RPM = 5676.0;
        double NEO_550_MAX_RPM = 11000.0;

        // Multiply percentage output (-1.0 to 1.0) by Max RPM
        double leftSimRPM = LeftIntakeLauncher.getAppliedOutput() * NEO_MAX_RPM;
        double rightSimRPM = RightIntakeLauncher.getAppliedOutput() * NEO_MAX_RPM;
        double indexerSimRPM = Indexer.getAppliedOutput() * NEO_550_MAX_RPM;

        // Feed the mocked RPM directly into the REV sim
        leftLauncherSim.iterate(leftSimRPM, batteryVoltage, kSimDt);
        rightLauncherSim.iterate(rightSimRPM, batteryVoltage, kSimDt);
        indexerSim.iterate(indexerSimRPM, batteryVoltage, kSimDt);

        SmartDashboard.putNumber("Sim/Fuel/LeftRPM", leftIntakeLauncherEncoder.getVelocity());
        SmartDashboard.putNumber("Sim/Fuel/RightRPM", rightIntakeLauncherEncoder.getVelocity());
        SmartDashboard.putNumber("Sim/Fuel/IndexerPercent", Indexer.getAppliedOutput());
    }
}
