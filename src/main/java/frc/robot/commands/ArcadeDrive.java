// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

/** The default drive command that uses the drive subsystem. */
public class ArcadeDrive extends Command {
    private final DriveSubsystem m_driveSubsystem;

    private final DoubleSupplier m_speedSupplier;
    private final DoubleSupplier m_rotationSupplier;

    /*private final DoubleSupplier m_left_y; // this gives us the left y axis for current controller
    private final DoubleSupplier m_right_x; // this gives us the right y axis for current controller
    private final BooleanSupplier m_left_b; // this gives us the left y axis for current controller
    private final BooleanSupplier m_b_button; // this gives us the right y axis for current controller
    private final BooleanSupplier m_x_button; // this gives us the left y axis for current controller
    private final BooleanSupplier m_a_button; // A button to reverse eject motor
    private final BooleanSupplier m_toggle_button; // A button to toggle the drive mode
*/
    /**
     * Creates a new DefaultDrive command.
     *
     * @param subsystem         The drive subsystem used by this command.
     * @param xbox_left_y       A function that returns the value of the left y axis
     *                          for the joystick.
     * @param xbox_right_x      A function that returns the value of the right Y
     *                          axis for the joystick.
     * @param xbox_left_b       A function that returns the value of the left y axis
     *                          for the joystick.
     * @param xbox_b_button
     * @param xbox_x_button
     * @param xbox_a_button
     * @param xbox_start_button
     *
     */
    public ArcadeDrive(DriveSubsystem subsystem, DoubleSupplier speed, DoubleSupplier rotation) {
        m_driveSubsystem = subsystem;

        m_speedSupplier = speed;
        m_rotationSupplier = rotation;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // we include a limit on the drivers speed for safety.
        // Additonally the axis's on the
        // publish raw button states for debugging
        //m_driveSubsystem.drive(m_left_b.getAsBoolean(), m_toggle_button.getAsBoolean(), m_left_y.getAsDouble(),
                //m_right_x.getAsDouble());

        double speed = m_speedSupplier.getAsDouble();
        double turn = m_rotationSupplier.getAsDouble();

        // 2. Apply Deadband (Crucial for preventing drift)
        speed = MathUtil.applyDeadband(speed, 0.1);
        turn = MathUtil.applyDeadband(turn, 0.1);

        // 4. (Optional) Square Inputs for finer control at low speeds
        // pure math requires keeping the sign (+/-)
        // speed = Math.copySign(speed * speed, speed);
        // turn = Math.copySign(turn * turn, turn);

        // 5. Send calculated values to subsystem
        m_driveSubsystem.drive(speed, turn);

    }

    // Called once the command ends or is interrupted.
    // @Override
    // public void end(boolean interrupted) {
    // m_driveSubsystem.stop(); // We might not want this
    // }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}