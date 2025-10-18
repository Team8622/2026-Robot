// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DEADBAND = 0.05;
  }

  public static class IntakeConstants {
    public static final int LEAD_MOTOR_CAN_ID = 13;
    public static final int FOLLOW_MOTOR_CAN_ID = 14;

    public static final double IN_SPEED = -0.25;
    public static final double OUT_SPEED = 0.25;

    public static final double OUT_SPEED_FAST = 0.6;
  }

  public static class AlgaeConstants {
    public static final int MOTOR_CAN_ID = 12;

    public static final double IN_SPEED = -0.25;
    public static final double OUT_SPEED = 0.45;
  }

  public static class ElevatorConstants {
    public static final int LEAD_MOTOR_CAN_ID = 10;
    public static final int FOLLOW_MOTOR_CAN_ID = 9;

    public static final double UP_SPEED = 0.1;
    public static final double DOWN_SPEED = -0.1;

    public static final double UP_SPEED_FAST = 0.4;
    public static final double DOWN_SPEED_FAST = -0.4;
  }

  public static class DriveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
  }
}
