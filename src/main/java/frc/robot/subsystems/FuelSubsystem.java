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
        // create brushed motors for each of the motors on the launcher mechanism
        LeftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        RightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);

        leftIntakeLauncherPID = LeftIntakeLauncher.getClosedLoopController();
        rightIntakeLauncherPID = RightIntakeLauncher.getClosedLoopController();

        leftIntakeLauncherEncoder = LeftIntakeLauncher.getEncoder();
        rightIntakeLauncherEncoder = RightIntakeLauncher.getEncoder();

        // create the configuration for the feeder roller, set a current limit and apply
        // the config to the controller
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
        feederConfig.inverted(false);
        Indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // create the configuration for the launcher roller, set a current limit, set
        // the motor to inverted so that positive values are used for both intaking and
        // launching, and apply the config to the controller
        SparkMaxConfig launcherConfig = new SparkMaxConfig();

        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig.p(0.0002);
        pidConfig.i(0);
        pidConfig.d(0);
        pidConfig.velocityFF(0.00018);

        launcherConfig.apply(pidConfig);

        launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        launcherConfig.voltageCompensation(12);
        launcherConfig.idleMode(IdleMode.kCoast);

        launcherConfig.inverted(true);
        RightIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        launcherConfig.inverted(false);
        LeftIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Motor sims
        DCMotor launcherMotor = DCMotor.getNEO(1);
        DCMotor indexerMotor = DCMotor.getNeo550(1);
        leftLauncherSim = new SparkMaxSim(LeftIntakeLauncher, launcherMotor);
        rightLauncherSim = new SparkMaxSim(RightIntakeLauncher, launcherMotor);
        indexerSim = new SparkMaxSim(Indexer, indexerMotor);

        // put default values for various fuel operations onto the dashboard
        // all commands using this subsystem pull values from the dashbaord to allow
        // you to tune the values easily, and then replace the values in Constants.java
        // with your new values. For more information, see the Software Guide.
        SmartDashboard.putNumber("Intaking feeder roller value",
                INDEXER_INTAKING_PERCENT);
        SmartDashboard.putNumber("Intaking intake roller value",
                INTAKE_INTAKING_PERCENT);
        SmartDashboard.putNumber("Launching feeder roller value",
                INDEXER_LAUNCHING_PERCENT);
        SmartDashboard.putNumber("Launching launcher roller value",
                LAUNCHING_LAUNCHER_PERCENT);
        // SmartDashboard.putNumber("Spin-up feeder roller value",
        // SPIN_UP_FEEDER_VOLTAGE);
    }

    // A method to set the voltage of the intake roller
    public void setIntakeLauncherRoller(double power) {
        LeftIntakeLauncher.set(power);
        RightIntakeLauncher.set(power); // positive for shooting
    }

    public void setShooterVoltage(double voltage) {
        leftIntakeLauncherPID.setSetpoint(voltage, SparkMax.ControlType.kVoltage);
        rightIntakeLauncherPID.setSetpoint(voltage, SparkMax.ControlType.kVoltage);
    }

    // A method to set the voltage of the intake roller
    public void setFeederRoller(double power) {
        Indexer.set(power); // positive for shooting
    }

    // A method to stop the rollers
    public void stop() {
        Indexer.set(0);
        LeftIntakeLauncher.set(0);
        RightIntakeLauncher.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RoboRioSim.getVInVoltage();

        leftLauncherSim.iterate(LeftIntakeLauncher.getAppliedOutput() * batteryVoltage, batteryVoltage, kSimDt);
        rightLauncherSim.iterate(RightIntakeLauncher.getAppliedOutput() * batteryVoltage, batteryVoltage, kSimDt);
        indexerSim.iterate(Indexer.getAppliedOutput() * batteryVoltage, batteryVoltage, kSimDt);

        double leftRPM = leftIntakeLauncherEncoder.getVelocity();
        double rightRPM = rightIntakeLauncherEncoder.getVelocity();
        double indexerPercent = Indexer.getAppliedOutput();

        SmartDashboard.putNumber("Sim/Fuel/LeftRPM", leftRPM);
        SmartDashboard.putNumber("Sim/Fuel/RightRPM", rightRPM);
        SmartDashboard.putNumber("Sim/Fuel/IndexerPercent", indexerPercent);
    }

    private boolean launcherAtSpeed(double targetRPM, double toleranceRPM) {
        double left = leftIntakeLauncherEncoder.getVelocity();
        double right = rightIntakeLauncherEncoder.getVelocity();
        return Math.abs(left - targetRPM) <= toleranceRPM && Math.abs(right - targetRPM) <= toleranceRPM;
    }

    // TODO: figure out the right rpm
    public Command shootAtSpeedCommand(double targetRPM, double toleranceRPM) {
        return Commands.sequence(
                    Commands.runOnce(() -> {
                        leftIntakeLauncherPID.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
                        rightIntakeLauncherPID.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
                    }, this),
                    Commands.waitUntil(() -> launcherAtSpeed(targetRPM, toleranceRPM)),
                    Commands.run(() -> setFeederRoller(INDEXER_LAUNCHING_PERCENT), this))
                .finallyDo(interrupted -> stop())
                .withName("ShootAtRPM");
    }
}