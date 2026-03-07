package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AlignBackward extends Command {

  private final DriveSubsystem m_driveSubsystem;

  public AlignBackward(DriveSubsystem driveSystem) {
    this.m_driveSubsystem = driveSystem;
    addRequirements(driveSystem);
  }

  @Override
  public void initialize() {
    // Optional: reset PID if needed
  }

  @Override
  public void execute() {
    m_driveSubsystem.alignToAngle(180);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop robot when command finishes
    m_driveSubsystem.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_driveSubsystem.isAligned();
  }
}