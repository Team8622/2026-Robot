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
import frc.robot.Constants.AlgaeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeLever extends SubsystemBase implements ExtendedSubsystem {
  SparkMax motor;
  boolean isActive;

  public AlgaeLever() {
    motor = new SparkMax(AlgaeConstants.MOTOR_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    isActive = false;
  }

  @Override
	public void periodic() {
		SmartDashboard.putBoolean("Algae Lever", isActive);
	}

  @Override
  public void start(double speed) {
		motor.set(speed);
		isActive = true;
  }

  @Override
  public void stop() {
    motor.set(0);
		isActive = false;
  }
}
