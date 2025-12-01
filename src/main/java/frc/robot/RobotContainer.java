// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AnalogCommand;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

	private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
	private final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

	private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
		() -> applyControllerRamp(driverController.getLeftY(), driverController.getLeftX()), // -1 on blue, 1 on red
		() -> applyControllerRamp(driverController.getLeftX(), driverController.getLeftY()))
		.withControllerRotationAxis(() -> driverController.getRightX() * -1)
		.deadband(ControllerConstants.DEADBAND)
		.scaleTranslation(Constants.DriveConstants.SCALE_TRANSLATION)
		.allianceRelativeControl(false);

	private final Command swerveDriveCommand = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

	private final SendableChooser<Command> autonomousChooser;

	// Change value based on if the code is being deployed for testing or for a competition match
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
	}

	private void configureBindings() {
	}

	private static double applyControllerRamp(double primary, double secondary) {
        double speed = Math.sqrt(Math.pow(primary, 2) + Math.pow(secondary, 2));
		double max = Math.max(Math.abs(primary), Math.abs(secondary));
		if (max == 0)
            return 0;

		double normalizedValue = primary * speed / max;

		double rampedValue = Math.signum(normalizedValue) * (1 - Math.sqrt(1 - Math.pow(normalizedValue, 2)));
		return rampedValue;
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autonomousChooser.getSelected();
	}
}
