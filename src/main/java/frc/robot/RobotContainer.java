package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.ArcadeDrive;

public class RobotContainer {

    // private final DefaultDrive m_defaultDrive =
    // new DefaultDrive(m_driveSubsystem, this::getControllerLeftY,
    // this::getControllerRightY);

        //private final XboxController gamepad0 = new XboxController(0);


    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final CommandXboxController m_driverController = new CommandXboxController(0);

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
            () -> -m_driverController.getRightX()
        ));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureBindings() {
        m_driverController.start()
            .onTrue(Commands.runOnce(m_driveSubsystem::toggleDriveMode, m_driveSubsystem));
        m_driverController.leftBumper()
            .onTrue(Commands.runOnce(m_driveSubsystem::toggleHalfSpeed, m_driveSubsystem));
    }
}
