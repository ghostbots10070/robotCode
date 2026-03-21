package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class AutoRoutines {
    private DriveSubsystem drivetrain;
    private ClimberSubsystem climberSubsystem;
    private FuelSubsystem fuelSubsystem;

    public AutoRoutines(DriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem, ClimberSubsystem climberSubsystem) {
        this.drivetrain = driveSubsystem;
        this.fuelSubsystem = ballSubsystem;
        this.climberSubsystem = climberSubsystem;
    }

    public Command basicAuto() {
        return Commands.sequence(
            new AutoDrive(drivetrain,0.10,  0.0).withTimeout(2),
            new LaunchSequence(fuelSubsystem).withTimeout(10)
        );
    }

    public Command encoderPositionAuto() {
        return Commands.sequence(
            drivetrain.movePositionCommand(1.5).withTimeout(10),
            new LaunchSequence(fuelSubsystem).withTimeout(10)
        );
    }
    public Command fullAuto() {
       return Commands.sequence(
            Commands.parallel(
                drivetrain.driveDistanceCommand(2.23, 0.1).withTimeout(7),
                climberSubsystem.prepClimber().withTimeout(5)
            ),
            fuelSubsystem.shootAtSpeedCommand().withTimeout(5),
            //Commands.runOnce(() -> fuelSubsystem.stop()).withTimeout(0.1),
            climberSubsystem.autoL1Climb().withTimeout(5)
        );

    }
    // public Command driveAndIntake() {
    //     return Commands.sequence(
    //         Commands.parallel(
    //             drivetrain.driveCommand(0.5, 0.5),
    //             intake.runIntakeCommand(1.0)
    //         ).withTimeout(5.0),
    //         Commands.parallel(
    //             drivetrain.stopCommand();
    //             intake.stopCommand();
    //         )
    //     );
    // }
    // public Command driveThenIntake() {
    //     return Commands.sequence(
    //         drivetrain.driveCommand(0.5, 0.5).withTimeout(5.0),
    //         drivetrain.stopCommand(),
    //         intake.runIntakeCommand(1.0).withTimeout(5.0),
    //         intake.stopCommand()
    //     );
    // }
}
