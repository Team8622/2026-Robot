
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsytem extends SubsystemBase implements ExtendedSubsystem {
	SparkMax leadMotor;
	SparkMax followMotor;

	boolean isActive;

	public ShooterSubsytem() {
		leadMotor = new SparkMax(ShooterConstants.LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
		followMotor = new SparkMax(ShooterConstants.FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);

		SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
		leadMotorConfig.idleMode(IdleMode.kBrake);
		leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkMaxConfig followMotorConfig = new SparkMaxConfig();
		followMotorConfig.idleMode(IdleMode.kBrake);
		followMotorConfig.follow(ShooterConstants.LEAD_MOTOR_CAN_ID, false);
		followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		isActive = false;
	}

	@Override
	public void periodic() {

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
