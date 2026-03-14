// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

//This was ripped from last years code, so don't trust it.
public class ZeroGyro extends Command {
  SwerveSubsystem m_swerve;

  public ZeroGyro(SwerveSubsystem swerve) {
    m_swerve = swerve;
    addRequirements(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.zeroGyroWithAlliance();
  }
}
