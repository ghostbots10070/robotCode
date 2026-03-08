package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
// import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.AlignStraight;
import frc.robot.commands.AlignBackward;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoRoutines;
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
    private final CommandXboxController m_operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    // Init For Autonomous
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        AutoRoutines routines = new AutoRoutines(m_driveSubsystem, m_fuelSubsystem);
        autoChooser.setDefaultOption("Basic HH Auto", routines.basicAuto());
        autoChooser.addOption("Encoder Position Auto", routines.encoderPositionAuto());


        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putNumber("heading", 0);

        SmartDashboard.putString("Speed Multiplier", "100%");

        NamedCommands.registerCommand("Shoot", new LaunchSequence(m_fuelSubsystem));

        m_driveSubsystem.setDefaultCommand(new ArcadeDrive(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getRightX()
        ));

        m_fuelSubsystem.setDefaultCommand(m_fuelSubsystem.run(() -> m_fuelSubsystem.stop()));

        m_climberSubsystem.setDefaultCommand(m_climberSubsystem.run(() -> m_climberSubsystem.stop()));
                
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureBindings() {
        DataLogManager.start();
        DataLogManager.log("Configuring button bindings");

        // Driver Controls
        m_driverController.start().onTrue(Commands.runOnce(m_driveSubsystem::toggleDriveMode, m_driveSubsystem));
        m_driverController.leftBumper().onTrue(Commands.runOnce(m_driveSubsystem::toggleHalfSpeed, m_driveSubsystem));
        m_driverController.back().onTrue(Commands.runOnce(m_driveSubsystem::toggleDirection, m_driveSubsystem));

        m_driverController.a().onTrue(m_driveSubsystem.setMaxSpeed(1.0));
        m_driverController.b().onTrue(m_driveSubsystem.setMaxSpeed(0.25));
        m_driverController.x().onTrue(m_driveSubsystem.setMaxSpeed(0.75));
        m_driverController.y().onTrue(m_driveSubsystem.setMaxSpeed(0.5));


        //m_driverController.a().whileTrue(new AlignStraight(m_driveSubsystem));
        //m_driverController.b().whileTrue(new AlignBackward(m_driveSubsystem));

        // sysid commands
        //m_driverController.y().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        //m_driverController.x().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        m_operatorController.a().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        m_operatorController.b().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // Operator Controls
        m_operatorController.leftTrigger().whileTrue(new Intake(m_fuelSubsystem)); // gets ready to intake stuff
        m_operatorController.rightBumper().whileTrue(new LaunchSequence(m_fuelSubsystem)); // launch balls
        m_operatorController.x().whileTrue(new Eject(m_fuelSubsystem)); // eject balls out of the intake
        m_operatorController.povDown().whileTrue(new ClimbDown(m_climberSubsystem));
        m_operatorController.povUp().whileTrue(new ClimbUp(m_climberSubsystem));
    }
}
