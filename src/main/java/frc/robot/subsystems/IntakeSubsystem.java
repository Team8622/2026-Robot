package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase implements ExtendedSubsystem, TogglableSubsystem {
	private SparkMax leadMotor;
	private SparkMax followMotor;

	private double currentSpeed;
	private boolean isActive;

	public IntakeSubsystem() {
		leadMotor = new SparkMax(IntakeConstants.LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
		followMotor = new SparkMax(IntakeConstants.FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);

		SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
		leadMotorConfig.idleMode(IdleMode.kBrake);
		leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		SparkMaxConfig followMotorConfig = new SparkMaxConfig();
		followMotorConfig.idleMode(IdleMode.kBrake);
		followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		currentSpeed = 0;
		isActive = false;
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("IntakeSubsystem", isActive);
	}

	@Override
	public void start(double speed) {
		leadMotor.set(speed);
        followMotor.set(speed * IntakeConstants.SPEED_RATIO);
		currentSpeed = speed;
		isActive = true;
	}

	@Override
	public void stop() {
		leadMotor.set(0);
        followMotor.set(0);
		currentSpeed = 0;
		isActive = false;
	}

	@Override
	public double getCurrentSpeed() {
		return currentSpeed;
	}
}
