// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/** The default drive command that uses the drive subsystem. */
public class ArcadeDrive extends Command {
    private final DriveSubsystem m_driveSubsystem;

    private final DoubleSupplier m_speedSupplier;
    private final DoubleSupplier m_rotationSupplier;

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
        // TODO: include a limit on the drivers speed for safety.
        double speed = m_speedSupplier.getAsDouble();
        double turn = m_rotationSupplier.getAsDouble();

        // 2. Apply Deadband (Crucial for preventing drift)
        speed = MathUtil.applyDeadband(speed, 0.1);
        turn = MathUtil.applyDeadband(turn, 0.1);

        // 4. (Optional) Square Inputs for finer control at low speeds
        // pure math requires keeping the sign (+/-)
        speed = Math.copySign(speed * speed, speed);
        turn = Math.copySign(turn * turn, turn);

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