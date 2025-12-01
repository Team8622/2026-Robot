
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NewSubsystemConstants;


public class NewSubsystem extends SubsystemBase implements ExtendedSubsystem {
    SparkMax leadMotor;
    SparkMax followMotor;
    boolean isActive;

	public NewSubsystem() {
        leadMotor = new SparkMax(NewSubsystemConstants.LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
		followMotor = new SparkMax(NewSubsystemConstants.FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);
        
		SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
		leadMotorConfig.idleMode(IdleMode.kCoast);
		leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followMotorConfig = new SparkMaxConfig();
		followMotorConfig.idleMode(IdleMode.kCoast);
        followMotorConfig.follow(NewSubsystemConstants.LEAD_MOTOR_CAN_ID, true);
		followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        isActive = false;
	}

	@Override
	public void periodic() {
        SmartDashboard.putBoolean("new subsystem", isActive);
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
