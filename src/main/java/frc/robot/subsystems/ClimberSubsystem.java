
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase implements ExtendedSubsystem {
	private SparkMax leadMotor;
	private SparkMax followMotor;

	private boolean isActive;

	public ClimberSubsystem() {
		leadMotor = new SparkMax(ClimberConstants.LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
		followMotor = new SparkMax(ClimberConstants.FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);

		SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
		leadMotorConfig.idleMode(IdleMode.kBrake);
		leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkMaxConfig followMotorConfig = new SparkMaxConfig();
        followMotorConfig.idleMode(IdleMode.kBrake);
        followMotorConfig.follow(ClimberConstants.LEAD_MOTOR_CAN_ID, true);
        followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		isActive = false;
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("ClimberSubsystem", isActive);
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
