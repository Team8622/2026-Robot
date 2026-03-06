
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
	private SparkMax motor;

	private boolean isActive;

	public ClimberSubsystem() {
		motor = new SparkMax(ClimberConstants.MOTOR_CAN_ID, MotorType.kBrushless);

		SparkMaxConfig motorConfig = new SparkMaxConfig();
		motorConfig.idleMode(IdleMode.kBrake);
		motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		isActive = false;
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("ClimberSubsystem", isActive);
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
