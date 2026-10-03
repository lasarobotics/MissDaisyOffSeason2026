// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIdentification {
    public static final int SLAP_DOWN_MOTOR_ID = 9;
    public static final int INTAKE_ROLLER_LEADER_MOTOR_ID = 10;
    public static final int INTAKE_ROLLER_FOLLOWER_MOTOR_ID = 11;
    public static final int FEEDING_ROLLER_MOTOR_ID = 12;
    public static final int SHOOTER_FEED_LEADER_MOTOR_ID = 13;
    public static final int SHOOTER_FEED_FOLLOWER_MOTOR_ID = 14;
    public static final int TURRET_MOTOR_ID = 15;
    public static final int SHOOTER_SPEED_LEADER_MOTOR_ID = 16;
    public static final int SHOOTER_SPEED_FOLLOWER_MOTOR_ID = 17;
    public static final int HOOD_ANGLE_MOTOR_ID = 18;
    public static final int ENCODER1 = 19;
    public static final int ENCODER2 = 20;
  }

  public static class DriveConstants {
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(3); // TODO measure
    public static final AngularVelocity MAX_ANGULAR_RATE =
        RotationsPerSecond.of(0.75); // TODO measure
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1); // TODO
    // measure

    public static final double STOW_DISTANCE_REQUIREMENT = 6.75;
    public static final double CENTER_XPOS = 8.25;

    public static final double DEADBAND_SCALAR = 0.1;
    public static final double SLOW_SPEED_SCALAR = 0.1;
    public static final double MID_SPEED_SCALAR = 0.5;
    public static final double FAST_SPEED_SCALAR = 0.75;

    public static final double SINGLE_TAG_AMBIGUITY_CUTOFF = 0.5;
    public static final double SINGLE_TAG_DISTANCE_CUTOFF = 5;
  }

  public static class IntakeConstants {
    public static final int SLAPDOWN_STOWED_POS = 0;
    public static final int SLAPDOWN_DOWN_POS = 0;
    public static final int INTAKE_ROLLER_MOTOR_SPEED = 0;
  }

  public static class SerializationConstants {
    public static final int FEEDING_ROLLER_MOTOR_SPEED = 0;
    public static final int SHOOTER_FEED_MOTOR_SPEED = 0;
  }

  public static class ShooterConstants {
    public static final int GEAR1_TEETH = 17;
    public static final int GEAR2_TEETH = 18;
    public static final int TURRET_TEETH = 92;

    public static final int ANGLE_LOWER_BOUND = -180;
    public static final int ANGLE_HIGHER_BOUND = 360;

    public static final double MOTOR_TURRET_GEAR_RATIO = -1;
    public static final double MOTOR_HOOD_GEAR_RATIO = -1;
    public static final double HANG_TIME = 2;
  }

  public static class HubConstants {
    public static final Translation2d BLUE_HUB_POS = new Translation2d(4.61, 4.021);
    public static final Translation2d RED_HUB_POS = new Translation2d(11.9, 4.021);
    public static final double BLUE_BUMP_XPOS = 0;
    public static final double RED_BUMP_XPOS = 0;
    public static final double LEFTY_POS = 0;
    public static final double RIGHTY_POS = 0;
  }

  public static final class BLineConstants {
    public static final double MAX_VELOCITY_MPS = 4.0;
    public static final double MAX_ACCELERATION_MPS2 = 3.0;
    public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 360.0;
    public static final double MAX_ANGULAR_ACCELERATION_DEG_PER_SEC2 = 720.0;
    public static final double END_TRANSLATION_TOLERANCE_METERS = 0.05;
    public static final double END_ROTATION_TOLERANCE_DEG = 2.0;
    public static final double INTERMEDIATE_HANDOFF_RADIUS_METERS = 0.30;

    public static final double TRANSLATION_KP = 5.0;
    public static final double TRANSLATION_KI = 0.0;
    public static final double TRANSLATION_KD = 0.0;

    public static final double ROTATION_KP = 3.0;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0;

    public static final double CROSS_TRACK_KP = 2.0;
    public static final double CROSS_TRACK_KI = 0.0;
    public static final double CROSS_TRACK_KD = 0.0;
  }

public static final class LimelightConstants {

  public static final String LIMELIGHT_NAME = "limelight";

  public static final double TURRET_PIVOT_FORWARD_METERS = 0.0;
  public static final double TURRET_PIVOT_LEFT_METERS = 0.0;
  public static final double TURRET_PIVOT_HEIGHT_METERS = 0.0;

  public static final double CAMERA_FORWARD_METERS = 0.0;
  public static final double CAMERA_LEFT_METERS = 0.0;
  public static final double CAMERA_HEIGHT_METERS = 0.0;

  public static final double CAMERA_YAW_AT_ZERO_DEG = 0.0;
  public static final double CAMERA_PITCH_DEG = 0.0;
  public static final double CAMERA_ROLL_DEG = 0.0;

  public static final double MAX_TAG_AMBIGUITY = 1.0;
  public static final double MAX_VISION_ANGULAR_VELOCITY_DEG_PER_SEC = 720.0;
  public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 0.0;

  public static final double VISION_STD_DEV_X = 0.7;
  public static final double VISION_STD_DEV_Y = 0.7;

  public static final double VISION_STD_DEV_THETA = 9999999.0;
}
}
