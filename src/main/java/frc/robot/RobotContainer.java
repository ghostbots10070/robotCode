package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
// import frc.robot.subsystems.CameraSubsystem;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;

import static frc.robot.Constants.OperatorConstants.*;

public class RobotContainer {

    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    // private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    private final FuelSubsystem m_fuelSubsystem = new FuelSubsystem();

    private final CommandXboxController m_driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    // private final CommandXboxController m_operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    // Init For Autonomous
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        // NamedCommands.registerCommand("EjectCommand", m_shooterCommand);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putNumber("heading", 0);

        m_driveSubsystem.setDefaultCommand(new ArcadeDrive(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getRightX()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureBindings() {
        DataLogManager.start();
        DataLogManager.log("Configuring button bindings");

        m_driverController.start().onTrue(Commands.runOnce(m_driveSubsystem::toggleDriveMode, m_driveSubsystem));
        
        m_driverController.leftBumper().onTrue(Commands.runOnce(m_driveSubsystem::toggleHalfSpeed, m_driveSubsystem));

        m_driverController.leftTrigger().whileTrue(new Intake(m_fuelSubsystem));

        m_driverController.rightBumper().whileTrue(new LaunchSequence(m_fuelSubsystem));

        m_driverController.x().whileTrue(new Eject(m_fuelSubsystem));

        m_driverController.povDown().whileTrue(new ClimbDown(m_climberSubsystem));

        m_driverController.povUp().whileTrue(new ClimbUp(m_climberSubsystem));
    }
}
