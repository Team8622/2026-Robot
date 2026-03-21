// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class TargetConstants{
		public static final Pose3d BLUE_HUB_POSE3D = new Pose3d(4.03,4,4.35, new Rotation3d());
		public static final Pose3d RED_HUB_POSE3D = new Pose3d(12.51,4,4.35,new Rotation3d());
	}

	public static class SimulationConstants{
		public static final Pose2d BLUE_SIM_STARTING_POSTIION = new Pose2d(new Translation2d(3.550,4), Rotation2d.fromDegrees(0));
        public static final Pose2d RED_SIM_STARTING_POSITION = new Pose2d(new Translation2d(12.99,4), Rotation2d.fromDegrees(180));
	}
	
	public static class ControllerConstants {
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int OPERATOR_CONTROLLER_PORT = 1;

		public static final double DEADBAND = 0.05;
	}

	public static class IntakeConstants {
		// public static final int LEAD_MOTOR_CAN_ID = 13;
		// public static final int FOLLOW_MOTOR_CAN_ID = 14;
		public static final int LEAD_MOTOR_CAN_ID = 14;

		public static final double FORWARD_SPEED = -0.5;
		public static final double BACKWARD_SPEED = 0.5;

		public static final double SPEED_RATIO = -1;
	}

	public static class ShooterConstants {
		public static final int UPPER_LEAD_MOTOR_CAN_ID = 9;
		public static final int LOWER_LEAD_MOTOR_CAN_ID = 10;
		public static final int UPPER_FOLLOW_MOTOR_CAN_ID = 11;
		public static final int LOWER_FOLLOW_MOTOR_CAN_ID = 12;

		public static final double FORWARD_SPEED = 1;
		public static final double BACKWARD_SPEED = -0.5;

		public static final double SPEED_RATIO = 0.5;
	}

	public static class DriveConstants {
		public static final double MAX_SPEED = Units.feetToMeters(10);
		public static final double SCALE_TRANSLATION = 0.8;
	}

	public static class RoboRIOSerialNumbers {
		public static final String COMP_CHASSIS = "03161778";
		public static final String TEST_CHASSIS = "03415940";
	}

	public static class LimelightConstants{
		public static final Pose3d LIMELIGHT_POSITION = new Pose3d(0.1460,0,0.4953,new Rotation3d());
	}
}
