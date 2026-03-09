// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ExtendedSubsystem;
import frc.robot.subsystems.TogglableSubsystem;

public class ToggleCommand extends Command {
	private final SubsystemBase subsystem;
	private final double speed;

	public ToggleCommand(SubsystemBase subsystem, double speed) {
		if (subsystem instanceof ExtendedSubsystem == false)
			throw new Error("Subsystems need to implement ExtendedSubsystem in order to use ToggleCommand");

		if (subsystem instanceof TogglableSubsystem == false)
			throw new Error("Subsystems need to implement TogglableSubsystem in order to use ToggleCommand");

		this.subsystem = subsystem;
		this.speed = speed;

		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		ExtendedSubsystem subsystem = (ExtendedSubsystem) this.subsystem;
		TogglableSubsystem togglableSubystem = (TogglableSubsystem) this.subsystem;

		if (togglableSubystem.getCurrentSpeed() != speed) {
			subsystem.start(speed);
		} else {
			subsystem.stop();
		}
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
