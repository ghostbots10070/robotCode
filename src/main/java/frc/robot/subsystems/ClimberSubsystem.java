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
import edu.wpi.first.wpilibj.Preferences;
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

    private static final String maxAngleKey = "ClimberMaxAngle";
    private static final String minAngleKey = "ClimberMinAngle";

    private static final String resetAngleKey = "ClimberResetAngle";
    private static final String prepAngleKey = "ClimberPrepAngle";
    private static final String l1ClimbAngleKey = "ClimberL1ClimbAngle";


    // these are the default values
    private double maxAngle = 170; // when the climber is inside the robot
    private double minAngle = 10.6; // when the climber is fully lowered

    // TODO: use preferences
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/robot-preferences.html#reading-preferences
    private double resetAngle = 99; // fully upwards
    private double prepAngle = 20; // the angle we want to be at to prep for climbing
    private double l1ClimbAngle = 99; // the angle we want to be at to be in position for an L1 climb

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

        Preferences.initDouble(maxAngleKey, maxAngle);
        Preferences.initDouble(minAngleKey, minAngle);
        Preferences.initDouble(resetAngleKey, resetAngle);
        Preferences.initDouble(prepAngleKey, prepAngle);
        Preferences.initDouble(l1ClimbAngleKey, l1ClimbAngle);

        refreshPreferences();

        rawClimberPotentiometer = new AnalogInput(CLIMBER_POTENTIOMETER_CHANNEL);
        climberPotentiometer = new AnalogPotentiometer(rawClimberPotentiometer, 180);
        SmartDashboard.putData("Climber/Raw", climberPotentiometer);
        SmartDashboard.putNumber("Climber/Angle", climberPotentiometer.get());

        SmartDashboard.putNumber("Climber/Prep Angle", prepAngle);
        SmartDashboard.putNumber("Climber/Reset Angle", resetAngle);
        SmartDashboard.putNumber("Climber/L1 Climb Angle", l1ClimbAngle);

        SmartDashboard.putBoolean("Climber/Safety Limiter", true);
    }

    public void refreshPreferences() {
        maxAngle = Preferences.getDouble(maxAngleKey, maxAngle);
        minAngle = Preferences.getDouble(minAngleKey, minAngle);
        resetAngle = Preferences.getDouble(resetAngleKey, resetAngle);
        prepAngle = Preferences.getDouble(prepAngleKey, prepAngle);
        l1ClimbAngle = Preferences.getDouble(l1ClimbAngleKey, l1ClimbAngle);

        SmartDashboard.putNumber("Climber/Prep Angle", prepAngle);
        SmartDashboard.putNumber("Climber/Reset Angle", resetAngle);
        SmartDashboard.putNumber("Climber/L1 Climb Angle", l1ClimbAngle);
    }

    // A method to set the percentage of the climber
    public void setClimber(double power) {
        lastSetPower = power; // Track last set power for tests
        climberMotor.set(power);
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
        return setToAngleCommand(prepAngle);
    }

    public Command autoL1Climb() {
        return setToAngleCommand(l1ClimbAngle);
    }

    public Command resetClimber() {
        return setToAngleCommand(resetAngle);
    }

    @Override
    public void periodic() {
        if (SmartDashboard.getBoolean("Climber/Safety Limiter", true)) {
            if (lastSetPower > 0 && climberPotentiometer.get() >= maxAngle) {
                lastSetPower = 0; // Prevent moving up if at or above max angle
                stop();
            } else if (lastSetPower < 0 && climberPotentiometer.get() <= minAngle) {
                lastSetPower = 0; // Prevent moving down if at or below min angle
                stop();
            }
        }


        // This method will be called once per scheduler run
        SmartDashboard.putData("Climber/Raw", climberPotentiometer);
       // SmartDashboard.putNumber("Climber/voltage", climberPotentiometer.getVoltage());
        //SmartDashboard.putNumber("Climber/avgVoltage", climberPotentiometer.getAverageVoltage());

        SmartDashboard.putNumber("Climber/Angle", climberPotentiometer.get());

        SmartDashboard.putNumber("Climber/RawVoltage", rawClimberPotentiometer.getVoltage());
        SmartDashboard.putNumber("Climber/RawAvgVoltage", rawClimberPotentiometer.getAverageVoltage());
    }
}