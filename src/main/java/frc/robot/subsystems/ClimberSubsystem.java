package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor;

    // Add fields for testing
    private double lastSetPower = 0;
    private boolean stopCalled = false;

    /** Creates a new CANBallSubsystem. */
    public ClimberSubsystem() {
        // create brushed motors for each of the motors on the launcher mechanism
        climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushed);

        // create the configuration for the climb moter, set a current limit and apply
        // the config to the controller
        SparkMaxConfig climbConfig = new SparkMaxConfig();
        climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
        climbConfig.idleMode(IdleMode.kBrake);
        climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // A method to set the percentage of the climber
    public void setClimber(double power) {
        climberMotor.set(power);
        lastSetPower = power; // Track last set power for tests
    }

    // A method to stop the climber
    public void stop() {
        climberMotor.set(0);
        stopCalled = true; // Track stop for tests
    }

    // Methods for tests to access state
    public double getLastSetPower() {
        return lastSetPower;
    }

    public boolean wasStopCalled() {
        return stopCalled;
    }

    public Command climbUpCommand() {
        return Commands.startEnd(
                () -> setClimber(CLIMBER_MOTOR_UP_PERCENT),
                this::stop,
                this);
    }

    public Command climbDownCommand() {
        return Commands.startEnd(
                () -> setClimber(CLIMBER_MOTOR_DOWN_PERCENT),
                this::stop,
                this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}