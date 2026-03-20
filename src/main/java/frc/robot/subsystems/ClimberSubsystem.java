package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor;

    // Add fields for testing
    private double lastSetPower = 0;
    private boolean stopCalled = false;

    private AnalogPotentiometer climberPotentiometer;
    private AnalogInput rawClimberPotentiometer;

    private double maxAngle = 170; // when the climber is inside the robot
    private double minAngle = 2; // when the climber is fully lowered

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


        rawClimberPotentiometer = new AnalogInput(CLIMBER_POTENTIOMETER_CHANNEL);
        climberPotentiometer = new AnalogPotentiometer(rawClimberPotentiometer, 180);
        SmartDashboard.putData("Climber/Raw", climberPotentiometer);

        // should be at zero degrees in the normal starting position
        SmartDashboard.putNumber("Climber/Angle", climberPotentiometer.get());
    }

    // A method to set the percentage of the climber
    public void setClimber(double power) {
        // if (power > 0 && climberPotentiometer.get() >= maxAngle) {
        //     power = 0; // Prevent moving up if at or above max angle
        // } else if (power < 0 && climberPotentiometer.get() <= minAngle) {
        //     power = 0; // Prevent moving down if at or below min angle
        // }

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

    public boolean shouldntGoDownMore() {
        double currentAngle = climberPotentiometer.get();
        return currentAngle <= minAngle;
    }

    public boolean shouldntGoUpMore() {
        double currentAngle = climberPotentiometer.get();
        return currentAngle >= maxAngle;
    }

    public Command setToAngleCommand(double targetAngle) {
        final boolean goingUp = targetAngle > climberPotentiometer.get();

        return Commands.run(() -> {
            double currentAngle = climberPotentiometer.get();
            

            if (goingUp && currentAngle >= maxAngle) {
                stop();
                return;
            } else if (!goingUp && currentAngle <= minAngle) {
                stop();
                return;
            }

            if (currentAngle < targetAngle) {
                setClimber(CLIMBER_MOTOR_UP_PERCENT);
            } else if (currentAngle > targetAngle) {
                setClimber(CLIMBER_MOTOR_DOWN_PERCENT);
            } else {
                stop();
            }
        }, this)
        .until(() -> Math.abs(climberPotentiometer.get() - targetAngle) <= 1.0)
        .finallyDo(interrupted -> stop());
    }

    public Command prepClimber() {
        return setToAngleCommand(20);
    }

    public Command autoL1Climb() {
        return setToAngleCommand(99);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putData("Climber/Raw", climberPotentiometer);
       // SmartDashboard.putNumber("Climber/voltage", climberPotentiometer.getVoltage());
        //SmartDashboard.putNumber("Climber/avgVoltage", climberPotentiometer.getAverageVoltage());

        SmartDashboard.putNumber("Climber/Angle", climberPotentiometer.get());

        SmartDashboard.putNumber("Climber/RawVoltage", rawClimberPotentiometer.getVoltage());
        SmartDashboard.putNumber("Climber/RawAvgVoltage", rawClimberPotentiometer.getAverageVoltage());
    }
}