// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ExtendedSubsystem;

public class AnalogCommand extends Command {
  SubsystemBase subsystem;
  double speed;
  boolean stopOnEnd;

  public AnalogCommand(SubsystemBase subsystem, double speed) {
    if (subsystem instanceof ExtendedSubsystem == false) throw new Error("Subsystems need to implement ExtendedSubsystem in order to use AnalogCommand");

    this.subsystem = subsystem;
    this.speed = speed;
    this.stopOnEnd = false;

    addRequirements(subsystem);
  }

  public AnalogCommand(SubsystemBase subsystem, double speed, boolean stopOnEnd) {
    if (subsystem instanceof ExtendedSubsystem == false) throw new Error("Subsystems need to implement ExtendedSubsystem in order to use AnalogCommand");

    this.subsystem = subsystem;
    this.speed = speed;
    this.stopOnEnd = stopOnEnd;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    ExtendedSubsystem subsystem = (ExtendedSubsystem) this.subsystem;

    if (speed != 0) {
      subsystem.start(speed);
    } else {
      subsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopOnEnd) return;

    ExtendedSubsystem subsystem = (ExtendedSubsystem) this.subsystem;
    subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return stopOnEnd;
  }
}
