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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.FuelConstants.*;

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
        pidConfig.p(0.0002);
        pidConfig.i(0);
        pidConfig.d(0);
        // In constructor
SmartDashboard.putNumber("Tuning/kP", 0.0002);
SmartDashboard.putNumber("Tuning/kI", 0.0);
SmartDashboard.putNumber("Tuning/kD", 0.0);
        //pidConfig.velocityFF(0.00018);

        launcherConfig.apply(pidConfig);
        launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        launcherConfig.voltageCompensation(12);
        launcherConfig.idleMode(IdleMode.kCoast);

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

    /** Commands both launcher motors to a target RPM using the onboard velocity PID. */
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

    /** Returns true when both launcher motors are within [tolerance] RPM of [targetRPM]. */
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
        return Commands.sequence(
            // Read targets once upfront so they can't drift mid-command
            Commands.runOnce(() -> {
                double targetRPM = SmartDashboard.getNumber(
                    "Launching launcher RPM", LAUNCHING_LAUNCHER_RPM);
                setShooterRPM(targetRPM);
            }, this),

            // Wait until flywheel is actually at speed before feeding anything
            Commands.waitUntil(() -> {
                double targetRPM = SmartDashboard.getNumber(
                    "Launching launcher RPM", LAUNCHING_LAUNCHER_RPM);
                double tolerance = SmartDashboard.getNumber(
                    "Launching RPM tolerance", LAUNCHING_RPM_TOLERANCE);
                return launcherAtRPM(targetRPM, tolerance);
            }),

            // Feed only when flywheel is at speed — pauses between balls to let it recover
            Commands.run(() -> {
                double targetRPM = SmartDashboard.getNumber(
                    "Launching launcher RPM", LAUNCHING_LAUNCHER_RPM);
                double tolerance = SmartDashboard.getNumber(
                    "Launching RPM tolerance", LAUNCHING_RPM_TOLERANCE);
                double feederPercent = SmartDashboard.getNumber(
                    "Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);

                if (launcherAtRPM(targetRPM, tolerance)) {
                    setFeederRoller(feederPercent); // up to speed → feed
                } else {
                    setFeederRoller(0);             // recovering → wait
                }
            }, this)
        )
        .finallyDo(interrupted -> stop())
        .withName("ShootAtRPM");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Fuel/Indexer Current", Indexer.getOutputCurrent());
        SmartDashboard.putNumber("Fuel/Left RPM", leftIntakeLauncherEncoder.getVelocity());
        SmartDashboard.putNumber("Fuel/Right RPM", rightIntakeLauncherEncoder.getVelocity());

        SmartDashboard.putBoolean("Fuel/Left Connected", LeftIntakeLauncher.getBusVoltage() > 1);
        SmartDashboard.putBoolean("Fuel/Right Connected", RightIntakeLauncher.getBusVoltage() > 1);
        SmartDashboard.putBoolean("Fuel/Indexer Connected", Indexer.getBusVoltage() > 1);

        // In periodic()
ClosedLoopConfig pidConfig = new ClosedLoopConfig();
pidConfig.p(SmartDashboard.getNumber("Tuning/kP", 0.0002));
pidConfig.i(SmartDashboard.getNumber("Tuning/kI", 0.0));
pidConfig.d(SmartDashboard.getNumber("Tuning/kD", 0.0));

SparkMaxConfig config = new SparkMaxConfig();
config.apply(pidConfig);
LeftIntakeLauncher.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
RightIntakeLauncher.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RoboRioSim.getVInVoltage();

        leftLauncherSim.iterate(LeftIntakeLauncher.getAppliedOutput() * batteryVoltage, batteryVoltage, kSimDt);
        rightLauncherSim.iterate(RightIntakeLauncher.getAppliedOutput() * batteryVoltage, batteryVoltage, kSimDt);
        indexerSim.iterate(Indexer.getAppliedOutput() * batteryVoltage, batteryVoltage, kSimDt);

        SmartDashboard.putNumber("Sim/Fuel/LeftRPM", leftIntakeLauncherEncoder.getVelocity());
        SmartDashboard.putNumber("Sim/Fuel/RightRPM", rightIntakeLauncherEncoder.getVelocity());
        SmartDashboard.putNumber("Sim/Fuel/IndexerPercent", Indexer.getAppliedOutput());
    }
}