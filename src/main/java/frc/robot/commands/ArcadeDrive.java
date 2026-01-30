// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;


/** The default drive command that uses the drive subsystem. */
public class ArcadeDrive extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_left_y; // this gives us the left y axis for current controller
  private final DoubleSupplier m_right_x; // this gives us the right y axis for current controller
  private final BooleanSupplier m_left_b; // this gives us the left y axis for current controller
  private final BooleanSupplier m_b_button; // this gives us the right y axis for current controller
  private final BooleanSupplier m_x_button; // this gives us the left y axis for current controller

  /**
   * Creates a new DefaultDrive command.
   *
   * @param d_subsystem The drive subsystem used by this command.
   * @param xbox_left_y A function that returns the value of the left y axis for the joystick.
   * @param xbox_right_x A function that returns the value of the right Y axis for the joystick.
   * @param xbox_left_b A function that returns the value of the left y axis for the joystick.
   * @param xbox_b_button
   * @param xbox_x_button
   */
  public ArcadeDrive(DriveSubsystem d_subsystem, DoubleSupplier xbox_left_y, DoubleSupplier xbox_right_x, BooleanSupplier xbox_left_b, BooleanSupplier xbox_b_button, BooleanSupplier xbox_x_button) {
    m_driveSubsystem = d_subsystem;
    m_left_y = xbox_left_y;
    m_right_x = xbox_right_x;
    m_left_b = xbox_left_b; // this gives us the left y axis for current controller
    m_b_button = xbox_b_button; // this gives us the right y axis for current controller
    m_x_button = xbox_x_button; // this gives us the left y axis for current controller
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // we include a limit on the drivers speed for safety.
    // Additonally the axis's on the
    m_driveSubsystem.drive(m_left_b.getAsBoolean(), m_x_button.getAsBoolean(), m_b_button.getAsBoolean(), m_left_y.getAsDouble(), m_right_x.getAsDouble());
  }

  // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_driveSubsystem.stop(); // We might not want this
//   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}