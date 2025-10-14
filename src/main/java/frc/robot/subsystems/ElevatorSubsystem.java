// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase implements ExtendedSubsystem {
  SparkMax leadMotor;
  SparkMax followMotor;
  boolean isActive;

  public ElevatorSubsystem() {
    leadMotor = new SparkMax(ElevatorConstants.LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
    followMotor = new SparkMax(ElevatorConstants.FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
    SparkMaxConfig followMotorConfig = new SparkMaxConfig();

    leadMotorConfig.idleMode(IdleMode.kBrake);
    leadMotorConfig.smartCurrentLimit(20);
    leadMotorConfig.voltageCompensation(12);

    followMotorConfig.idleMode(IdleMode.kBrake);
    followMotorConfig.smartCurrentLimit(20);
    followMotorConfig.voltageCompensation(12);
    followMotorConfig.follow(ElevatorConstants.LEAD_MOTOR_CAN_ID, true);

    leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    isActive = false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elevator", isActive);
  }

  @Override
  public void start(double speed) {
    leadMotor.set(speed);
    isActive = true;
  }

  @Override
  public void stop() {
    leadMotor.set(0);
    isActive = false;
  }
}
