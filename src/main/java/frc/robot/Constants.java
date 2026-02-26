package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The DriveConstants class has all of the constants needed for autonomous /
 * gyro assisted movement
 * of the robot. This class is kept seperate as its values change quite a lot.
 * Odemetry Is used to
 * track the position and state of the robot. Kinemtatics is used to calculate
 * how much power to
 * apply to each motor.
 */
public final class Constants {
    // general drive constants
    // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
    // https://sciencing.com/convert-rpm-linear-speed-8232280.html
    public static final class DriveConstants {
        // Default path config from path planning app
        public static RobotConfig autoConfig;

        static {
            try {
                autoConfig = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                // Throw an actual exception so the robot doesn't silently boot with a broken auto config
                throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings!", e);
            }
        }

        // Motor controller IDs for drivetrain motors
        public static final int LEFT_LEADER_ID = 1;
        public static final int LEFT_FOLLOWER_ID = 2;
        public static final int RIGHT_LEADER_ID = 3;
        public static final int RIGHT_FOLLOWER_ID = 4;

        // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
        // likelihood of tripping breakers or damaging CIM motors
        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // meters
        public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2;
        public static final double kTrackwidthMeters = 0.5461; // TODO: Update
        // this is not used and is handled by the rev encoder.
        public static final double PULSES_PER_REV = 1;
        public static final double GEAR_RATIO = 8.46; // 8.46:1
        // basically converted from rotations to to radians to then meters using the
        // wheel diameter.
        // the diameter is already *2 so we don't need to multiply by 2 again.
        public static final double POSITION_CONVERSION_RATIO = (Math.PI * WHEEL_DIAMETER) / PULSES_PER_REV / GEAR_RATIO;
        public static final double VELOCITY_CONVERSION_RATIO = POSITION_CONVERSION_RATIO / 60;
        // Kinematic constants

        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        // Feed Forward Constants
        public static final double ksDriveVolts = 0.0; ///0.1945
        public static final double kvDriveVoltSecondsPerMeter = 2.2623;
        public static final double kaDriveVoltSecondsSquaredPerMeter = 0.43785;
        public static final double kvDriveVoltSecondsPerMeterAngular = 1.2;
        public static final double kaDriveVoltSecondsSquaredPerMeterAngular = 0.2;
        // Max speed Constants
        public static final double kMaxOutputDrive = 0.9;
        public static final double kMinOutputDrive = -0.9;
        // Feed Back / PID Constants
        public static final double kPDriveVel = 0.00088622;
        public static final double kIDriveVel = 0.0;
        public static final double kDDriveVel = 0.0;
        public static final double kIzDriveVel = 0.0; // error before integral takes effect

        public static final double kPDrivePos = 8; // 4.6269
        public static final double kIDrivePos = 0.0;
        public static final double kDDrivePos = 0.49649;
        public static final double kIzDrivePos = 0.0; // error before integral takes effect
        // Helper class that converts a chassis velocity (dx and dtheta components) to
        // left and right
        // wheel velocities for a differential drive.

        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI; // 2π rad/s
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        // Motor Controller PID Slots
        public static final ClosedLoopSlot kDrivetrainVelocityPIDSlot = ClosedLoopSlot.kSlot0;
        public static final ClosedLoopSlot kDrivetrainPositionPIDSlot = ClosedLoopSlot.kSlot1;
    }

    public static final class FuelConstants {
        // Motor controller IDs for Fuel Mechanism motors
        public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 5;
        public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 6;
        public static final int INDEXER_MOTOR_ID = 8;

        // Current limit for fuel mechanism motors.
        public static final int INDEXER_MOTOR_CURRENT_LIMIT = 80;
        public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 80;

        // All values likely need to be tuned based on your robot
        public static final double INDEXER_INTAKING_PERCENT = -.8;
        public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
        public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;

        public static final double INTAKE_INTAKING_PERCENT = 0.6;
        public static final double LAUNCHING_LAUNCHER_PERCENT = .85;
        public static final double INTAKE_EJECT_PERCENT = -0.8;

        public static final double SPIN_UP_SECONDS = 0.75;
    }

    public static final class ClimbConstants {
        // Motor controller IDs for Climb motor
        public static final int CLIMBER_MOTOR_ID = 7;

        // Current limit for climb motor
        public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;
        // Percentage to power the motor both up and down
        public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.8;
        public static final double CLIMBER_MOTOR_UP_PERCENT = 0.8;
    }

    public static final class OperatorConstants {

        // Port constants for driver and operator controllers. These should match the
        // values in the Joystick tab of the Driver Station software
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // This value is multiplied by the joystick value when rotating the robot to
        // help avoid turning too fast and beign difficult to control
        public static final double DRIVE_SCALING = 0.7;
        public static final double ROTATION_SCALING = 0.8;
    }
}
