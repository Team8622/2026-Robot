// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase implements ExtendedSubsystem {
  SparkMax leadMotor;
  SparkMax followMotor;
  boolean isActive;

  public CoralIntake() {
    leadMotor = new SparkMax(IntakeConstants.LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
    followMotor = new SparkMax(IntakeConstants.FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
    leadMotorConfig.idleMode(IdleMode.kCoast);
    leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig followMotorConfig = new SparkMaxConfig();
    followMotorConfig.idleMode(IdleMode.kCoast);
    followMotorConfig.follow(IntakeConstants.LEAD_MOTOR_CAN_ID, true);
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    isActive = false;
  }

  @Override
	public void periodic() {
		SmartDashboard.putBoolean("Coral Intake", isActive);
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
