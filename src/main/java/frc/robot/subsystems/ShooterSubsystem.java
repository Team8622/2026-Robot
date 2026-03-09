package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public final class ShooterSubsystem extends SubsystemBase implements ExtendedSubsystem {
    private final SparkMax upperLeadMotor;
    private final SparkMax lowerLeadMotor;
    private final SparkMax upperFollowMotor;
    private final SparkMax lowerFollowMotor;

    private boolean isActive;

    public ShooterSubsystem() {
        upperLeadMotor = new SparkMax(ShooterConstants.UPPER_LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
        lowerLeadMotor = new SparkMax(ShooterConstants.LOWER_LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
        upperFollowMotor = new SparkMax(ShooterConstants.UPPER_FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);
        lowerFollowMotor = new SparkMax(ShooterConstants.LOWER_FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig upperLeadMotorConfig = new SparkMaxConfig();
        upperLeadMotorConfig.idleMode(IdleMode.kBrake);
        upperLeadMotor.configure(upperLeadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig lowerLeadMotorConfig = new SparkMaxConfig();
        lowerLeadMotorConfig.idleMode(IdleMode.kBrake);
        lowerLeadMotor.configure(lowerLeadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig upperFollowMotorConfig = new SparkMaxConfig();
        upperFollowMotorConfig.idleMode(IdleMode.kBrake);
        upperFollowMotorConfig.follow(ShooterConstants.UPPER_LEAD_MOTOR_CAN_ID, true);
        upperFollowMotor.configure(upperFollowMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig lowerFollowMotorConfig = new SparkMaxConfig();
        lowerFollowMotorConfig.idleMode(IdleMode.kBrake);
        lowerFollowMotorConfig.follow(ShooterConstants.LOWER_LEAD_MOTOR_CAN_ID, true);
        lowerFollowMotor.configure(lowerFollowMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        isActive = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ShooterSubsystem", isActive);
    }

    @Override
    public void start(double speed) {
        upperLeadMotor.set(speed * ShooterConstants.SPEED_RATIO);
        lowerLeadMotor.set(speed);
        isActive = true;
    }

    @Override
    public void stop() {
        upperLeadMotor.set(0);
        lowerLeadMotor.set(0);
        isActive = false;
    }
}
