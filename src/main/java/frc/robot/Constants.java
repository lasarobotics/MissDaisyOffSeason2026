// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

  public static class Drive {
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;

    public static final double WHEEL_FROM_CENTER_DIST = Inches.of(15.476).in(Meters);

    public static final AngularVelocity MAX_ANGULAR_RATE =
        RotationsPerSecond.of(
            MAX_SPEED.in(MetersPerSecond) / (2 * Math.PI * WHEEL_FROM_CENTER_DIST));

    public static final double STOW_DISTANCE_REQUIREMENT = 6.75;
    public static final double CENTER_XPOS = 8.25;

    public static final double DEADBAND_SCALAR = 0.1;
    public static final double SLOW_SPEED_SCALAR = 0.1;
    public static final double MID_SPEED_SCALAR = 0.5;
    public static final double FAST_SPEED_SCALAR = 0.75;

    public static final double SINGLE_TAG_AMBIGUITY_CUTOFF = 0.5;
    public static final double SINGLE_TAG_DISTANCE_CUTOFF = 5;
    public static final double ROTATION_DEADBAND = 0.002 * Math.PI * 2;
  }

  public static class Intake {
    public static final int ARM_CAN_ID = 30;
    public static final int LEADER_CAN_ID = 31;
    public static final int FOLLOWER_CAN_ID = 32;

    public static final Angle ARM_STOW_SETPOINT = Degrees.of(0.0);
    public static final Angle ARM_DEPLOY_SETPOINT = Degrees.of(0.0);
    public static final AngularVelocity INTAKE_ACTIVE_SPEED = RotationsPerSecond.of(0);
    public static final Distance INTAKE_ROLLER_DIAMETER = Meters.of(0.035);

    public static final AngularVelocity INTAKE_ROLLER_SPEED =
        RotationsPerSecond.of(
            (2 * Drive.MAX_SPEED.in(MetersPerSecond))
                / (Math.PI * INTAKE_ROLLER_DIAMETER.in(Meters)));

    public static final double ZERO_VOLTAGE = 3.0;
    public static final double ZERO_THRESHOLD = 3.0;

    public static final double ZERO_SECONDS_WAIT = 0.2;
  }

  public static class Serialization {
    public static final int OMNI_CAN_ID = 40;
    public static final int MECANUM_LEADER_CAN_ID = 41;
    public static final int MECANUM_FOLLOWER_CAN_ID = 42;
    public static final Distance OMNI_WHEEL_DIAMETER = Inches.of(6);
    public static final Distance MECANUM_WHEEL_DIAMETER = Inches.of(2.25);
    public static final LoggedNetworkNumber BALL_SPEED_INCREASER =
        new LoggedNetworkNumber("/Tuning/ballSpeedIncreaser", 0.1);

    public static final Supplier<AngularVelocity> MECANUM_SPEED =
        () -> {
          return RotationsPerSecond.of((2 * Drive.MAX_SPEED.in(MetersPerSecond) + BALL_SPEED_INCREASER.getAsDouble())/(Math.PI * OMNI_WHEEL_DIAMETER.in(Meters)));
        };
    public static final Supplier<AngularVelocity> OMNI_SPEED =
        () -> {
          return RotationsPerSecond.of((2 * Drive.MAX_SPEED.in(MetersPerSecond) + 2 * BALL_SPEED_INCREASER.getAsDouble())/(Math.PI * MECANUM_WHEEL_DIAMETER.in(Meters)));
        };
    public static final AngularVelocity MECANUM_REST_SPEED = RotationsPerSecond.of(0);
    public static final AngularVelocity OMNI_REST_SPEED = RotationsPerSecond.of(0);
  }

  public static class Shooter {
    public static final int SHOOTER_LEADER_ID = 50;
    public static final int SHOOTER_FOLLOWER_ID = 51;
    public static final int HOOD_MOTOR_ID = 52;
    public static final int TURRET_MOTOR_ID = 53;
    public static final int ENCODER_ONE_TEETH = 17;
    public static final int ENCODER_TWO_TEETH = 18;
    public static final int TURRET_GEAR_TEETH = 92;

    public static final int ENCODER_TWO_ID = 54;
    public static final int ENCODER_ONE_ID = 55;
    public static final double CRT_EPSILON = 0.01;
    public static final Distance SHOOTER_OFFSET_X = Inches.of(-4);
    public static final Distance SHOOTER_OFFSET_Y = Inches.of(-3);
    public static final Distance SHOOTER_OFFSET_RADIUS =
        Meters.of(Math.hypot(SHOOTER_OFFSET_X.in(Inches), SHOOTER_OFFSET_Y.in(Inches)));
    public static final Distance CENTER_TO_EDGE = Inches.of(13.25);
    public static final double HANG_TIME = 1.0;
    public static final double HOOD_COLLISION_TIME = 0.25;
    public static final double HUB_HEIGHT = 1.83;
    public static final LoggedNetworkNumber MAX_BALL_Y_POS =
        new LoggedNetworkNumber("Tuning/maxBallYPos", 3.0);
    public static final double HOOD_MAX_ANGLE = (19.0 / 175.0); // rotations
    public static final double TURRET_DEADBAND =
        0.035; // in radians. This is around 2 degrees, which is not terrible accuracy even at 12
    // meters away
    public static final double ZERO_VOLTAGE = 3.0;
    public static final double ZERO_THRESHOLD = 3.0;
    public static final double ZERO_SECONDS_WAIT = 0.2;
  }

  public static class Field {
    public static final Distance FIELD_X = Inches.of(650.12);
    public static final Distance FIELD_Y = Inches.of(316.64);
    public static final double NZ_MID_LINE_X = FIELD_X.in(Meters) / 2;
    public static final double NZ_MID_LINE_Y = FIELD_Y.in(Meters) / 2;
    public static final double HUB_WIDTH = Inches.of(47).in(Meters);
    public static final double AZ_DEPTH = Inches.of(158.6).in(Meters);

    public static final Translation3d BLUE_HUB_POS =
        new Translation3d(AZ_DEPTH + HUB_WIDTH / 2, NZ_MID_LINE_Y, Shooter.HUB_HEIGHT);
    public static final Translation3d RED_HUB_POS =
        new Translation3d(
            FIELD_X.in(Meters) - AZ_DEPTH - HUB_WIDTH / 2, NZ_MID_LINE_Y, Shooter.HUB_HEIGHT);
    public static final Translation3d BLUE_LEFT_BUMP =
        new Translation3d(BLUE_HUB_POS.getX(), 6.03, 0);
    public static final Translation3d BLUE_RIGHT_BUMP =
        new Translation3d(BLUE_HUB_POS.getX(), 1, 0);
    public static final Translation3d RED_LEFT_BUMP =
        new Translation3d(RED_HUB_POS.getX(), 2.01, 0);
    public static final Translation3d RED_RIGHT_BUMP =
        new Translation3d(RED_HUB_POS.getX(), 6.03, 0);

    public static final double NZ_RED_X = RED_HUB_POS.getX();
    public static final double NZ_BLUE_X = BLUE_HUB_POS.getX();

    public static final double TRENCH_WIDTH = Inches.of(50.34).in(Meters);

    public static final Translation2d RED_LEFT_TRENCH_P1 =
        new Translation2d(RED_HUB_POS.getX(), FIELD_Y.in(Meters));
    public static final Translation2d RED_LEFT_TRENCH_P2 =
        new Translation2d(RED_HUB_POS.getX(), FIELD_Y.in(Meters) - TRENCH_WIDTH);

    public static final Translation2d RED_RIGHT_TRENCH_P1 =
        new Translation2d(RED_HUB_POS.getX(), 0);
    public static final Translation2d RED_RIGHT_TRENCH_P2 =
        new Translation2d(RED_HUB_POS.getX(), TRENCH_WIDTH);

    public static final Translation2d BLUE_RIGHT_TRENCH_P1 =
        new Translation2d(BLUE_HUB_POS.getX(), 0);
    public static final Translation2d BLUE_RIGHT_TRENCH_P2 =
        new Translation2d(BLUE_HUB_POS.getX(), TRENCH_WIDTH);

    public static final Translation2d BLUE_LEFT_TRENCH_P1 =
        new Translation2d(BLUE_HUB_POS.getX(), FIELD_Y.in(Meters));
    public static final Translation2d BLUE_LEFT_TRENCH_P2 =
        new Translation2d(BLUE_HUB_POS.getX(), FIELD_Y.in(Meters) - TRENCH_WIDTH);

    // corner a and corner b of blue tower
    public static final Rectangle2d BLUE_TOWER =
        new Rectangle2d(new Translation2d(0, 3.173), new Translation2d(1.108, 4.346));
    public static final Rectangle2d RED_TOWER =
        new Rectangle2d(
            new Translation2d(FIELD_X.in(Meters), 3.173),
            new Translation2d(FIELD_X.in(Meters) - 1.108, 4.346));
    public static final double GRAVITY_VALUE = 9.81;
  }

  public static final class BLine {
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

  public static final class Limelight {

    public static final String LIMELIGHT_NAME = "limelight";

    public static final Distance TURRET_PIVOT_FORWARD = Inches.of(-4);
    public static final Distance TURRET_PIVOT_LEFT = Inches.of(-3);
    public static final Distance TURRET_PIVOT_HEIGHT = Inches.of(-5.445);

    public static final Distance CAMERA_FORWARD = Inches.of(-8.17);
    public static final Distance CAMERA_LEFT = Meters.of(0.0);
    public static final Distance CAMERA_HEIGHT = Inches.of(4.56);

    public static final Angle CAMERA_YAW_AT_ZERO = Degrees.of(25.006);
    public static final Angle CAMERA_PITCH = Degrees.of(0.0);
    public static final Angle CAMERA_ROLL = Degrees.of(0.0);

    public static final double MAX_TAG_AMBIGUITY = 1.0;
    public static final Angle MAX_VISION_ANGULAR_VELOCITY_PER_SEC = Degrees.of(720.0);
    public static final Distance MAX_SINGLE_TAG_DISTANCE = Meters.of(0.0);

    public static final double VISION_STD_DEV_X = 0.7;
    public static final double VISION_STD_DEV_Y = 0.7;

    public static final double VISION_STD_DEV_THETA = 9999999.0;
  }
}
