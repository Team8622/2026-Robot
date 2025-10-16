// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AnalogCommand;
import frc.robot.subsystems.AlgaeLever;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static final CoralIntake coralIntake = new CoralIntake();
  public static final AlgaeLever algaeLever = new AlgaeLever();
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private static final CommandXboxController driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
	private static final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  private static final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
    () -> driverController.getLeftY(), // -1 on blue, 1 on red
    () -> driverController.getLeftX())
    .withControllerRotationAxis(() -> driverController.getRightX() * -1)
    .deadband(ControllerConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(false
    );

  private static final Command swerveDriveCommand = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  private final SendableChooser<Command> autonomousChooser;

  // Change value based on if the code is being deployed for testing or for a competition match
  private final boolean isCompetition = false;

  public RobotContainer() {
    autonomousChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );
		SmartDashboard.putData(autonomousChooser);

    configureBindings();

    swerveSubsystem.setDefaultCommand(swerveDriveCommand);
  }

  private void configureBindings() {
    operatorController.a().whileTrue(new AnalogCommand(coralIntake, IntakeConstants.IN_SPEED));
    operatorController.b().whileTrue(new AnalogCommand(coralIntake, IntakeConstants.OUT_SPEED));
    operatorController.x().whileTrue(new AnalogCommand(coralIntake, IntakeConstants.OUT_SPEED_FAST));
    
    operatorController.povUp().whileTrue(new AnalogCommand(algaeLever, AlgaeConstants.IN_SPEED));
    operatorController.povDown().whileTrue(new AnalogCommand(algaeLever, AlgaeConstants.OUT_SPEED));

    operatorController.rightBumper().whileTrue(new AnalogCommand(elevatorSubsystem, ElevatorConstants.UP_SPEED));
    operatorController.leftBumper().whileTrue(new AnalogCommand(elevatorSubsystem, ElevatorConstants.DOWN_SPEED));
    operatorController.rightTrigger().whileTrue(new AnalogCommand(elevatorSubsystem, ElevatorConstants.UP_SPEED_FAST));
    operatorController.leftTrigger().whileTrue(new AnalogCommand(elevatorSubsystem, ElevatorConstants.DOWN_SPEED_FAST));

    driverController.rightTrigger().whileTrue(new AnalogCommand(coralIntake, IntakeConstants.IN_SPEED));
    driverController.leftTrigger().whileTrue(new AnalogCommand(coralIntake, IntakeConstants.OUT_SPEED));
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
