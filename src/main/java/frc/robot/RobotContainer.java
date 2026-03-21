// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AnalogCommand;
import frc.robot.commands.ToggleCommand;
import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(this);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
            () -> driverController.getLeftY() * (driverController.rightTrigger().getAsBoolean() ? 0.25 : 1),
            () -> driverController.getLeftX() * (driverController.rightTrigger().getAsBoolean() ? 0.25 : 1))
            .withControllerRotationAxis(() -> driverController.getRightX())
            .deadband(ControllerConstants.DEADBAND)
            .scaleTranslation(Constants.DriveConstants.SCALE_TRANSLATION)
            .robotRelative(false)
            .allianceRelativeControl(false);

    private final Command swerveDriveCommand = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    private final SendableChooser<Command> autonomousChooser;

    // Change value based on if the code is being deployed for testing or for a
    // competition match
    private final boolean isCompetition = false;

    public RobotContainer() {
        setupPathPlannerCommands();

        autonomousChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> isCompetition
                        ? stream.filter(auto -> auto.getName().startsWith("comp"))
                        : stream);
        SmartDashboard.putData(autonomousChooser);

        configureBindings();

        swerveSubsystem.setDefaultCommand(swerveDriveCommand);
    }

    private void setupPathPlannerCommands() {
        NamedCommands.registerCommand("Enable Shooter",
                new AnalogCommand(shooterSubsystem, ShooterConstants.FORWARD_SPEED, true));
        NamedCommands.registerCommand("Disable Shooter", new AnalogCommand(shooterSubsystem, 0, true));
        NamedCommands.registerCommand("Enable Intake",
                new AnalogCommand(intakeSubsystem, IntakeConstants.FORWARD_SPEED, true));
        NamedCommands.registerCommand("Disable Intake", new AnalogCommand(intakeSubsystem, 0, true));

        Pose2d hubShootingPointTargetPose = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) ? (new Pose2d(13.54, 4, Rotation2d.fromDegrees(180))) : ((new Pose2d(3, 4, Rotation2d.fromDegrees(0))));

        PathConstraints basicConstraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                hubShootingPointTargetPose,
                basicConstraints,
                0.0
        );
        NamedCommands.registerCommand("Pathfind to Hub", pathfindingCommand );

    }

    private void configureBindings() {
        operatorController.rightTrigger()
                .whileTrue(new AnalogCommand(shooterSubsystem, ShooterConstants.FORWARD_SPEED));
        operatorController.rightBumper()
                .whileTrue(new AnalogCommand(shooterSubsystem, ShooterConstants.BACKWARD_SPEED));

        operatorController.leftTrigger().onTrue(new ToggleCommand(intakeSubsystem, IntakeConstants.FORWARD_SPEED));
        operatorController.leftBumper().onTrue(new ToggleCommand(intakeSubsystem, IntakeConstants.BACKWARD_SPEED));

        driverController.y().whileTrue(new ZeroGyro(swerveSubsystem));
    }

    public boolean shouldAim() {
        return driverController.b().getAsBoolean();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }

    public SwerveSubsystem getSwerveSubSystem(){
        return swerveSubsystem;
    }
}
